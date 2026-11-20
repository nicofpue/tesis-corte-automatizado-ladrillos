#include "motor3.h"
#include "motor.h"
#include "encoder.h"
#include "pid_posicion.h"
#include "telemetria.h"
#include <math.h>
#include <stdio.h>

namespace {
const AxisConfig config=motor3Config();
Motor motor(config);
Encoder encoder(config);
PositionPid pid(config.pid);
enum class State { Idle, Moving, Settling, Stopped, Fault };
State state=State::Idle;
enum class Mode { Relative, SeekHome, SeekEnd };
Mode mode=Mode::Relative;
bool homeReferenced=false;
bool limitStable=false;
uint32_t limitSince=0;
bool initialized=false;
float distanceMm=0;
int8_t direction=1;
uint32_t startedAt=0,settledAt=0,feedbackAt=0,previousLoopUs=0;
uint32_t previousControlUs=0;
bool positionSettling=false;
int32_t previousCount=0;
int32_t startCount=0;
const char* message="posicion local desde encendido";
bool home() { return digitalRead(config.homePin)==config.limitActiveLevel; }
bool end() { return digitalRead(config.endPin)==config.limitActiveLevel; }
float displacement() {
    return static_cast<float>(static_cast<int64_t>(encoder.count())-startCount)*config.encoderSign*config.mmPerCount();
}
void fail(const char* reason) {
    motor.stop();state=State::Fault;message=reason;
    char text[170];snprintf(text,sizeof(text),"FALLO M3: %s; PWM=0; desplazamiento=%.3f mm",reason,displacement());printReply(text);
}
void finish(const char* reason,uint32_t now) {
    motor.stop();message=reason;settledAt=now;state=State::Settling;
}
const char* stateName() {
    switch(state) {
    case State::Idle:return "LISTO";case State::Moving:return "MOVIENDO";
    case State::Settling:return "ESTABILIZANDO";case State::Stopped:return "DETENIDO";
    default:return "FALLO";
    }
}
}
namespace Motor3 {
bool begin() {
    if (!isfinite(config.maxPwm)||config.maxPwm<=0||config.maxPwm>80||
        !isfinite(config.pid.kp)||config.pid.kp<=0||!isfinite(config.pid.ki)||config.pid.ki<0||
        !isfinite(config.pid.kd)||config.pid.kd<0||!isfinite(config.pid.integralLimit)||config.pid.integralLimit<0||
        !isfinite(config.pid.derivativeFilterS)||config.pid.derivativeFilterS<0||
        !isfinite(config.toleranceMm)||config.toleranceMm<=0||
        !isfinite(config.settledSpeedMmS)||config.settledSpeedMmS<=0||
        config.controlPeriodUs==0||config.controlPeriodUs>config.maxControlGapUs||
        (config.compensateDeadzone&&(!isfinite(config.deadzonePwm)||config.deadzonePwm<0||
          config.deadzonePwm>=config.maxPwm||!isfinite(config.minMovePwm)||
          config.minMovePwm<config.deadzonePwm||config.minMovePwm>config.maxPwm))||
        !isfinite(config.maxTravelMm)||config.maxTravelMm<=0||
        !isfinite(config.travelMarginMm)||config.travelMarginMm<0||config.countsPerRev<=0||
        config.leadMmRev<=0||config.encoderToOutputRatio<=0||!isfinite(config.mmPerCount())||
        config.mmPerCount()<=0||(config.encoderSign!=1&&config.encoderSign!=-1)||
        !isfinite(config.homePwm)||config.homePwm<=0||config.homePwm>config.maxPwm||
        !isfinite(config.endSeekPwm)||config.endSeekPwm<=0||config.endSeekPwm>config.maxPwm||
        config.limitConfirmMs==0||config.homeConfirmTimeoutMs<=config.limitConfirmMs||
        config.homeTimeoutMs<=config.limitConfirmMs||
        config.moveTimeoutMs<=config.settlingMs||config.noCountsTimeoutMs==0) {
        fail("configuracion invalida");return false;
    }
    if (!motor.begin()) { fail("no se pudo configurar PWM");return false; }
    pinMode(config.homePin,config.limitInputMode);pinMode(config.endPin,config.limitInputMode);
    encoder.begin();initialized=true;
    if (home()&&end()) { fail("HOME y END simultaneos");return false; }
    return true; // No hace HOME ni inicia movimiento.
}
bool busy() { return state==State::Moving||state==State::Settling; }
void stop() {
    motor.stop();
    if (state!=State::Fault) { state=State::Stopped;message="cancelado; no reanuda automaticamente"; }
}
bool move(float mm,int8_t sign) {
    if (!initialized||busy()) { printReply("M3 no inicializado u ocupado; S cancela");return false; }
    if (!isfinite(mm)||mm<=0||mm>config.maxTravelMm||(sign!=1&&sign!=-1)) {
        printReply("Usar M3,distancia_positiva_mm,1 o M3,distancia_positiva_mm,-1 dentro del limite");return false;
    }
    if (home()&&end()) { fail("HOME y END simultaneos");return false; }
    if ((sign<0&&home())||(sign>0&&end())) {
        printReply(sign<0?"M3: HOME activo bloquea retorno; se permite avance":"M3: END activo bloquea avance; se permite retorno");return false;
    }
    mode=Mode::Relative;
    motor.stop();startCount=encoder.count();distanceMm=mm;direction=sign;
    startedAt=feedbackAt=millis();
    previousControlUs=previousLoopUs=micros();
    previousCount=startCount;
    positionSettling=false;
    pid.reset(0);
    state=State::Moving;
    message="PID de posicion relativa";
    char text[170];
    snprintf(text,sizeof(text),"M3: desplazamiento solicitado=%+.3f mm; PID con techo=%.1f%%; maximo=%.0f s; sin HOME",
        direction*distanceMm,config.maxPwm,config.moveTimeoutMs/1000.0f);
    printReply(text);
    return true;
}
bool seekLimit(int8_t sign) {
    if (!initialized||busy()||(sign!=-1&&sign!=1)) {
        printReply("H3/E3: M3 no inicializado u ocupado; S cancela");
        return false;
    }
    if (home()&&end()) { fail("HOME y END simultaneos");return false; }
    motor.stop();
    mode=sign<0?Mode::SeekHome:Mode::SeekEnd;
    if (sign<0) homeReferenced=false;
    direction=sign;
    distanceMm=0;
    startCount=previousCount=encoder.count();
    startedAt=feedbackAt=millis();
    previousLoopUs=micros();
    state=State::Moving;
    message=sign<0?"buscando HOME":"buscando END";
    if (sign<0?home():end()) {
        limitStable=false;
        finish(message,startedAt);
    }
    char text[140];
    snprintf(text,sizeof(text),"M3 %s: buscando %s; PWM fijo %.1f%% si el final no esta activo",
        sign<0?"H3":"E3",sign<0?"HOME":"END",sign<0?config.homePwm:config.endSeekPwm);
    printReply(text);
    return true;
}
void update() {
    if (!initialized||!busy()) return;
    uint32_t now=millis(),us=micros();
    if (uint32_t(us-previousLoopUs)>config.maxControlGapUs) { fail("demora del bucle excedida");return; }
    previousLoopUs=us;
    if (home()&&end()) { fail("HOME y END simultaneos");return; }
    float delta=displacement(),progress=direction*delta;
    if (fabsf(delta)>config.maxTravelMm+config.travelMarginMm) { fail("recorrido por orden excedido");return; }
    const uint32_t timeout=mode==Mode::SeekHome?config.homeTimeoutMs:config.moveTimeoutMs;
    if (uint32_t(now-startedAt)>=timeout) { fail("tiempo maximo excedido");return; }
    if (mode!=Mode::Relative) {
        const bool limitActive=direction<0?home():end();
        if (state==State::Moving&&limitActive) {
            limitStable=false;
            finish(message,now);
        }
        if (state==State::Settling) {
            // Cortar al primer flanco y confirmar parado. No reanudar por rebote.
            if (uint32_t(now-settledAt)>=config.homeConfirmTimeoutMs) {
                fail("final M3 no se estabiliza");
                return;
            }
            if (!limitActive) { limitStable=false;return; }
            if (!limitStable) { limitStable=true;limitSince=now; }
            if (uint32_t(now-limitSince)<config.limitConfirmMs) return;
            if (mode==Mode::SeekHome) {
                encoder.zero();
                startCount=encoder.count();
                homeReferenced=true;
            }
            state=State::Stopped;
            message=mode==Mode::SeekHome?"HOME confirmado; x_local=0":"END confirmado";
            printReply(message);
            return;
        }
    }
    if (state==State::Settling) {
        motor.stop();
        if (direction<0&&home()) message="parado por HOME";
        if (direction>0&&end()) message="parado por END";
        if (uint32_t(now-settledAt)>=config.settlingMs) {
            state=State::Stopped;
            char text[180];snprintf(text,sizeof(text),"M3 FIN: %s; solicitado=%+.3f real_encoder=%+.3f mm; x_local=%.3f mm; PWM=0",
                message,direction*distanceMm,delta,encoder.position());printReply(text);
        }
        return;
    }
    // Supervisar el sentido aplicado incluso entre muestras del PID.
    if (motor.appliedPwm()<0&&home()) { finish("parado por HOME",now);return; }
    if (motor.appliedPwm()>0&&end()) { finish("parado por END",now);return; }
    if (progress < -config.travelMarginMm) { fail("encoder avanza contrario al sentido solicitado");return; }
    int32_t count=encoder.count();
    if (count!=previousCount||motor.appliedPwm()==0) { previousCount=count;feedbackAt=now; }
    else if (uint32_t(now-feedbackAt)>=config.noCountsTimeoutMs) { fail("sin cambio de cuentas con PWM activo");return; }
    if (mode!=Mode::Relative) {
        motor.drive(direction*(direction<0?config.homePwm:config.endSeekPwm),us);
        return;
    }
    const uint32_t elapsed=us-previousControlUs;
    if (elapsed<config.controlPeriodUs) return;
    if (elapsed>config.maxControlGapUs) { fail("periodo PID excedido");return; }
    previousControlUs=us;

    const float target=direction*distanceMm;
    const float limit=config.maxPwm-(config.compensateDeadzone?config.deadzonePwm:0);
    float output=pid.update(target,delta,elapsed/1000000.0f,limit);
    if (!isfinite(output)) { fail("salida PID no finita");return; }
    if (fabsf(target-delta)<=config.toleranceMm) {
        motor.stop();
        if (fabsf(pid.velocity())>config.settledSpeedMmS) { positionSettling=false;return; }
        if (!positionSettling) { positionSettling=true;settledAt=now; }
        if (uint32_t(now-settledAt)>=config.settlingMs) {
            state=State::Stopped;
            message="objetivo PID estabilizado";
            char text[180];
            snprintf(text,sizeof(text),"M3 FIN: %s; solicitado=%+.3f real_encoder=%+.3f mm; x_local=%.3f mm; PWM=0",
                message,target,delta,encoder.position());
            printReply(text);
        }
        return;
    }
    positionSettling=false;
    // Si hay sobrepaso, el PID puede corregir en sentido opuesto.
    if (output<0&&home()) { finish("parado por HOME",now);return; }
    if (output>0&&end()) { finish("parado por END",now);return; }
    if (output!=0&&config.compensateDeadzone) {
        const float magnitude=fmaxf(fabsf(output)+config.deadzonePwm,config.minMovePwm);
        output=copysignf(fminf(magnitude,config.maxPwm),output);
    }
    motor.drive(output,us);
}
bool reportEssential() {
    char text[180];
    const char* operation=mode==Mode::SeekHome?"H3":mode==Mode::SeekEnd?"E3":"PID";
    snprintf(text,sizeof(text),"M3 %s %s: x_local=%.3f mm ref=%d delta=%+.3f mm PWM=%.1f%% HOME=%d END=%d",
        stateName(),operation,encoder.position(),homeReferenced,displacement(),motor.appliedPwm(),home(),end());
    return printReply(text);
}
bool reportPins() {
    char text[180];snprintf(text,sizeof(text),"M3 PINES PWM=%u DIR=%u ENC=%u/%u HOME=%u END=%u; techo=%.1f%% escala=%.7f; Kp=%.3f Ki=%.3f Kd=%.3f",
        config.pwmPin,config.dirPin,config.encA,config.encB,config.homePin,config.endPin,config.maxPwm,config.mmPerCount(),config.pid.kp,config.pid.ki,config.pid.kd);return printReply(text);
}
bool reportEncoder() { return printEncoderDiagnostic(encoder.diagnostic(),config); }
}
