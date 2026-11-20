#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include "eje.h"
#include "comandos.h"
#include "geometria.h"
#include "telemetria.h"
#include "motor3.h"
#include "panel.h"
#include "config_panel.h"

const AxisConfig config1=motor1Config(),config2=motor2Config(),config3=motor3Config();
const AxisConfig* const configs[]={&config1,&config2};
const GeometryConfig geometry=geometryConfig();
Axis motor1(config1),motor2(config2);
Axis* const axes[]={&motor1,&motor2};
Commands commands;
enum class ProcessState { NotHomed, Homing, Ready, Positioning, Translating, Settling, Held, SensorStopped, Stopped, Fault };
enum class Reason { None, Contact, End };
ProcessState process=ProcessState::NotHomed;
struct StopEvent {
    Reason reason=Reason::None;
    bool confirmedContact=false,lossPending=false;
    uint32_t lossAt=0;
    int32_t counts[2]={};
    float x[2]={}; // Foto de AMBOS encoders cuando se detuvo este eje.
    uint32_t at=0;
} events[2];
float startPosition[2]={};
bool initialized=false;
char processMessage[160]="Inicializando";
uint32_t previousLoopUs=0,settlingAt=0;
int reportStep=-1;
bool encoderReport=false;
uint32_t reportAt=0;
struct ContactConfirmation { bool active=false,confirmed=false; uint32_t since=0; } contact[2];
struct FaultSnapshot {
    bool valid=false, transferTimedOut=false;
    bool contact[2]={};
    float x[2]={},target[2]={},pwm[2]={};
    float syncBoost[2]={};
    EncoderDiagnostic encoder[2];
} faultSnapshot;
struct HeldTransfer {
    bool active=false,paused=false,rawMissing=false;
    int syncPausedAxis=-1;
    float start[2]={},shift=0,referenceGap=0;
    uint32_t startedAt=0,lossAt=0,pauseAt=0;
} transfer;
int helpStep=-1;
uint32_t helpAt=0;
void fault(const char* text,int axis=-1);
void requestHelp() { helpStep=0;helpAt=millis()-REPORT_INTERVAL_MS; }
void serviceHelp() {
    static const char* lines[]={
        "--- COMANDOS (mm; traslados con pieza sujeta) ---",
        "H / L       Abrir y referenciar M1/M2",
        "H3 / E3     M3 a HOME / END con PWM fijo",
        "D,mm        Cierre con PWM fijo; cada CONTACTO detiene su eje",
        "C           Centrar pieza con PWM limitado (centro estimado)",
        "T,+mm/-mm   Trasladar: positivo derecha, negativo izquierda",
        "M3,mm,1/-1  Desplazamiento relativo M3 con PID",
        "Q           Estado esencial, separacion y margen izquierda/derecha",
        "E           Diagnostico completo de encoders y fallos",
        "ON / OFF    Contactor sierra: encender sin temporizador / apagar",
        "S / X       Parar tres motores y contactor | ? Ayuda",
        "START/STOP/EMERGENCIA fisicos: solo avisos; corte externo"
    };
    if (helpStep<0||uint32_t(millis()-helpAt)<REPORT_INTERVAL_MS) return;
    if (static_cast<unsigned>(helpStep)>=sizeof(lines)/sizeof(lines[0])) { helpStep=-1;return; }
    if (printReply(lines[helpStep])) { ++helpStep;helpAt=millis(); }
}

void requestReport(bool encoderOnly=false) { encoderReport=encoderOnly;reportStep=0;reportAt=millis()-REPORT_INTERVAL_MS; }
void notifyEvent() { printReply(processMessage);if (AUTOMATIC_FULL_REPORTS) requestReport(true); }
void serviceContacts() {
    for (int i=0;i<2;++i) {
        auto& c=contact[i];
        if (!axes[i]->contactPressed()) { c=ContactConfirmation{};continue; }
        if (!c.active) { c.active=true;c.since=millis(); }
        c.confirmed=uint32_t(millis()-c.since)>=configs[i]->limitConfirmMs;
    }
}
bool bothContactsConfirmed() { return contact[0].confirmed&&contact[1].confirmed; }
bool contactCorrectionAllowed() {
    return bothContactsConfirmed()&&motor1.isHomed()&&motor2.isHomed()&&
        process!=ProcessState::Homing&&process!=ProcessState::Fault;
}
bool printSeparation() {
    if (!motor1.isHomed()||!motor2.isHomed()) return printReply("Separacion no valida hasta completar HOME de ambos");
    char text[180];
    float reference=faceGap(geometry,motor1.position(),motor2.position());
    if (contactCorrectionAllowed()) {
        float corrected=reference+geometry.contactOffsetMm;
        if (corrected>0)
            snprintf(text,sizeof(text),"Separacion_corregida=%.3f mm (REF %.3f; desfase_total=%+.3f mm; ambos CONTACTO confirmados)",
                corrected,reference,geometry.contactOffsetMm);
        else snprintf(text,sizeof(text),"REF_carros=%.3f mm; correccion da valor no positivo: revisar calibracion",reference);
    } else snprintf(text,sizeof(text),"Separacion_REF=%.3f mm (sin correccion; CONTACTO confirmado=%d/%d)",
        reference,contact[0].confirmed,contact[1].confirmed);
    return printReply(text);
}
bool currentShiftRange(ShiftRange& r) {
    if (!motor1.isHomed()||!motor2.isHomed()||
        !translationRange(motor1.position(),motor2.position(),config1.maxTravelMm,config2.maxTravelMm,r)) return false;
    // Recortar por sensores activos segun el sentido fisico del traslado.
    if (motor1.homePressed()||motor2.endPressed()) r.minimum=fmaxf(r.minimum,0.0f);
    if (motor1.endPressed()||motor2.homePressed()) r.maximum=fminf(r.maximum,0.0f);
    return r.minimum<=r.maximum;
}
bool printTravelRange() {
    ShiftRange r;char text[180];
    if (!currentShiftRange(r)) return printReply("Margen traslado: no valido; completar/revisar HOME y recorridos");
    if (contactCorrectionAllowed()) {
        auto g=estimatedGrip(geometry,motor1.position(),motor2.position());
        snprintf(text,sizeof(text),"Traslado nominal: IZQ=%.3f DER=%.3f mm; centrar_EST=%+.3f mm; reparto I/D=%.0f/%.0f%%",
            fmaxf(0,-r.minimum),fmaxf(0,r.maximum),-g.center,100*geometry.leftContactShare,100*(1-geometry.leftContactShare));
    } else snprintf(text,sizeof(text),"Traslado nominal: IZQ=%.3f DER=%.3f mm; C/T requieren HOME valido y ambos CONTACTO",
        fmaxf(0,-r.minimum),fmaxf(0,r.maximum));
    return printReply(text);
}
void setMessage(const char* text) { snprintf(processMessage,sizeof(processMessage),"%s",text); }
void stopAll() { Panel::off();for (auto* a:axes) a->stop();Motor3::stop();transfer.active=false;transfer.paused=false; }
bool hasSensorStop() { return events[0].reason!=Reason::None||events[1].reason!=Reason::None; }
const char* reasonName(Reason r) { return r==Reason::Contact?"CONTACTO":r==Reason::End?"END":"NINGUNO"; }
void captureStopSnapshot(bool transferTimedOut) {
    faultSnapshot.valid=true;
    faultSnapshot.transferTimedOut=transferTimedOut;
    for (int i=0;i<2;++i) {
        faultSnapshot.x[i]=axes[i]->position();faultSnapshot.target[i]=axes[i]->target();
        faultSnapshot.contact[i]=transferTimedOut&&axes[i]->contactPressed();
        faultSnapshot.pwm[i]=axes[i]->hasFault()?axes[i]->faultPwm():axes[i]->pwm();
        faultSnapshot.syncBoost[i]=axes[i]->hasFault()?axes[i]->faultSyncBoost():axes[i]->syncBoost();
        faultSnapshot.encoder[i]=axes[i]->hasFault()?axes[i]->faultEncoderDiagnostic():axes[i]->encoderDiagnostic();
    }
}
void fault(const char* text,int axis) {
    captureStopSnapshot(false);
    stopAll();process=ProcessState::Fault;
    if (axis<0) snprintf(processMessage,sizeof(processMessage),"FALLO: %s; todos PWM=0",text);
    else snprintf(processMessage,sizeof(processMessage),"FALLO MOTOR %d: %s; todos PWM=0",axis+1,text);
    notifyEvent();
}
bool validPins() {
    const uint8_t pins[]={config1.pwmPin,config1.dirPin,config1.encA,config1.encB,config1.homePin,config1.endPin,config1.contactPin,
        config2.pwmPin,config2.dirPin,config2.encA,config2.encB,config2.homePin,config2.endPin,config2.contactPin,
        config3.pwmPin,config3.dirPin,config3.encA,config3.encB,config3.homePin,config3.endPin,
        PanelConfig::contactorPin,PanelConfig::startPin,PanelConfig::stopPin,PanelConfig::emergencyPin,PanelConfig::rgbPin};
    for (unsigned i=0;i<sizeof(pins);++i) {
        if (pins[i]==255) return false;
        for (unsigned j=i+1;j<sizeof(pins);++j) if (pins[i]==pins[j]) return false;
    }
    return config1.pwmChannel!=config2.pwmChannel&&config1.pwmChannel!=config3.pwmChannel&&config2.pwmChannel!=config3.pwmChannel;
}
bool busy() { return process==ProcessState::Homing||process==ProcessState::Positioning||process==ProcessState::Translating; }
void startHome(bool automatic=false) {
    if (!initialized) { printReply("Inicializacion incompleta; corregir y reiniciar");return; }
    // H/L es explicito: interrumpe la operacion actual y abre ambos ejes.
    for (int i=0;i<2;++i) {
        // Solo el arranque automatico exige estar sin objeto. H/L explicito puede liberar.
        if (automatic&&axes[i]->contactPressed()) { fault("CONTACTO al encender: HOME automatico bloqueado; H/L permite abrir",i);return; }
        if (axes[i]->homePressed()&&axes[i]->endPressed()) { fault("HOME y END simultaneos",i);return; }
    }
    stopAll();
    faultSnapshot=FaultSnapshot{};
    for (auto& e:events) e=StopEvent{};
    for (auto& c:contact) c=ContactConfirmation{};
    for (int i=0;i<2;++i) if (!axes[i]->home()) { fault("No se pudo iniciar HOME",i);return; }
    process=ProcessState::Homing;previousLoopUs=micros();
    setMessage("HOME M1/M2: permite abrir con END/CONTACTO activos; M3 detenido; H3 referencia M3");printReply(processMessage);
}
void recordSensorStop(int i,Reason reason) {
    if (events[i].reason!=Reason::None) return;
    // Detener SOLO este eje. El otro conserva su objetivo y su PID.
    if (reason==Reason::Contact) axes[i]->haltContact(); else axes[i]->haltEnd();
    auto& e=events[i];e.reason=reason;e.at=millis();
    for (int j=0;j<2;++j) {
        e.counts[j]=axes[j]->counts();
        e.x[j]=configs[j]->encoderSign*static_cast<float>(e.counts[j])*configs[j]->mmPerCount();
    }
    snprintf(processMessage,sizeof(processMessage),"MOTOR %d parado por %s; el otro eje continua si no ha terminado",i+1,reasonName(reason));
    notifyEvent();
}
void startSeparation(float gap) {
    if (Motor3::busy()) { printReply("M3 ocupado; esperar o S para cancelar antes de D");return; }
    if (!initialized||process!=ProcessState::Ready) { printReply("D requiere HOME completo y estado LISTO; H/L para referenciar");return; }
    float target[2];
    if (!separationTargets(geometry,gap,config1.maxTravelMm,config2.maxTravelMm,target[0],target[1])) {
        char text[160];
        float minimum=2*fmaxf(0.0f,fmaxf(geometry.leftHomeToCenterMm-config1.maxTravelMm,geometry.rightHomeToCenterMm-config2.maxTravelMm));
        float maximum=2*fminf(geometry.leftHomeToCenterMm,geometry.rightHomeToCenterMm);
        snprintf(text,sizeof(text),"D fuera de recorrido respecto al centro fijo; intervalo nominal [%.3f, %.3f] mm",minimum,maximum);
        printReply(text);return;
    }
    for (auto& e:events) e=StopEvent{};
    transfer.active=false;
    for (int i=0;i<2;++i) {
        startPosition[i]=axes[i]->position();
        if (!axes[i]->approachTo(target[i])) { fault("No se pudo iniciar posicion",i);return; }
    }
    process=ProcessState::Positioning;previousLoopUs=micros();
    snprintf(processMessage,sizeof(processMessage),"Objetivo separacion=%.3f mm; x1=%.3f x2=%.3f mm; centro fijo; maximo=%.0f/%.0f s",
        gap,target[0],target[1],config1.moveTimeoutMs/1000.0f,config2.moveTimeoutMs/1000.0f);printReply(processMessage);
}
void startTransfer(float shift,bool center) {
    if (!initialized||busy()||process==ProcessState::Settling||Motor3::busy()||
        process==ProcessState::Stopped||process==ProcessState::Fault) {
        printReply("C/T: esperar fin de operacion; tras S/fallo completar H y sujecion");return;
    }
    if (!contactCorrectionAllowed()) { printReply("C/T requiere HOME valido y ambos CONTACTO confirmados");return; }
    auto g=estimatedGrip(geometry,motor1.position(),motor2.position());
    if (!isfinite(g.center)||!isfinite(g.width)||g.width<=0) { printReply("Geometria de pieza invalida");return; }
    if (center) shift=-g.center;
    if (isfinite(shift)&&fabsf(shift)<=geometry.centerToleranceMm) {
        if (!transfer.active) {
            transfer=HeldTransfer{};transfer.active=true;
            transfer.referenceGap=faceGap(geometry,motor1.position(),motor2.position());
        }
        process=ProcessState::Held;
        printReply(center?"Centro ESTIMADO ya dentro de tolerancia; PWM=0":"Traslado dentro de tolerancia; sin movimiento");
        printTravelRange();return;
    }
    ShiftRange range;
    if (!isfinite(shift)||!currentShiftRange(range)||shift<range.minimum||shift>range.maximum) {
        printReply("Traslado fuera de recorrido/sensores; no se inicia movimiento");printTravelRange();return;
    }
    transfer=HeldTransfer{};
    transfer.start[0]=motor1.position();transfer.start[1]=motor2.position();
    transfer.referenceGap=faceGap(geometry,transfer.start[0],transfer.start[1]);
    transfer.shift=shift;transfer.startedAt=millis();
    if (!motor1.moveHeldTo(transfer.start[0]+shift)) { fault(motor1.message(),0);return; }
    if (!motor2.moveHeldTo(transfer.start[1]-shift)) { fault(motor2.message(),1);return; }
    transfer.active=true;process=ProcessState::Translating;previousLoopUs=micros();
    snprintf(processMessage,sizeof(processMessage),"TRASLADO=%+.3f mm (+derecha); objetivos M1=%.3f M2=%.3f; PWM max %.1f/%.1f%%; plazo=%lu ms",
        shift,motor1.target(),motor2.target(),config1.heldMaxPwm,config2.heldMaxPwm,
        static_cast<unsigned long>(geometry.heldTimeoutMs));notifyEvent();
}
void stopTransferByTime(uint32_t elapsedMs) {
    captureStopSnapshot(true);
    stopAll();process=ProcessState::Stopped;
    snprintf(processMessage,sizeof(processMessage),"TRASLADO INCOMPLETO: %+.3f mm pedidos; %lu ms (plazo %lu ms); PWM=0; H para referenciar",
        transfer.shift,static_cast<unsigned long>(elapsedMs),static_cast<unsigned long>(geometry.heldTimeoutMs));
    requestReport();
}
bool superviseHeldTransfer(bool finishing=false) {
    if (!transfer.active) return true;
    for (int i=0;i<2;++i) {
        if (axes[i]->hasFault()) { fault(axes[i]->message(),i);return false; }
        if (!axes[i]->isHomed()) { fault("Referencia HOME perdida en sujecion",i);return false; }
    }
    uint32_t now=millis();
    const bool inTransfer=process==ProcessState::Translating||process==ProcessState::Settling;
    const uint32_t elapsedMs=now-transfer.startedAt;
    if (inTransfer&&elapsedMs>=geometry.heldTimeoutMs) { stopTransferByTime(elapsedMs);return false; }
    serviceContacts();
    bool raw=motor1.contactPressed()&&motor2.contactPressed();
    if (!raw) {
        for (auto* a:axes) a->pauseHeldMotion();
        bool thirdMoving=Motor3::busy();
        Motor3::stop(); // Si estaba trabajando con pieza sujeta, no continua al perderla.
        if (thirdMoving) printReply("M3 detenido por perdida de CONTACTO; requiere nueva orden");
        if (!transfer.paused) { transfer.paused=true;transfer.pauseAt=now; }
        if (!transfer.rawMissing) { transfer.rawMissing=true;transfer.lossAt=now; }
        if (uint32_t(now-transfer.lossAt)>=geometry.contactLossConfirmMs) fault("CONTACTO perdido durante sujecion/traslado");
        return false;
    }
    transfer.rawMissing=false;
    if (transfer.paused) {
        for (auto* a:axes) a->pauseHeldMotion();
        if (!bothContactsConfirmed()) {
            if (uint32_t(now-transfer.pauseAt)>=geometry.contactRecoverTimeoutMs) fault("CONTACTOS no se estabilizan");
            return false;
        }
        transfer.paused=false;
    }
    const float gap=faceGap(geometry,motor1.position(),motor2.position());
    const float gapChange=fabsf(gap-transfer.referenceGap);
    if ((!inTransfer||finishing)&&gapChange>geometry.maxHeldGapChangeMm) {
        char text[120];
        snprintf(text,sizeof(text),"Cambio excesivo de separacion con pieza sujeta: %.3f mm (limite %.3f mm)",
            gapChange,geometry.maxHeldGapChangeMm);
        fault(text);return false;
    }
    if (finishing) {
        for (int i=0;i<2;++i) {
            if (fabsf(axes[i]->position()-axes[i]->target())>axes[i]->config().toleranceMm) {
                fault("Traslado fuera de la posicion objetivo tras estabilizar",i);return false;
            }
        }
    }
    for (auto* a:axes) a->synchronization(0,0);
    if (process==ProcessState::Translating) {
        int8_t dir=transfer.shift>0?1:-1;
        float error=((motor1.position()-transfer.start[0])+(motor2.position()-transfer.start[1]))*dir;
        const int ahead=error>0 ? 0 : 1;
        const float pauseThreshold=transfer.syncPausedAxis==ahead
            ? geometry.maxHeldGapChangeMm : geometry.heldSyncPauseMm;
        transfer.syncPausedAxis=-1;
        Axis& leading=*axes[ahead];
        const int8_t leadingDir=ahead==0 ? dir : -dir;
        const float remaining=(leading.target()-leading.position())*leadingDir;
        // Permitir que el adelantado se estabilice o corrija en sentido inverso.
        if (fabsf(error)>pauseThreshold&&leading.busy()&&axes[1-ahead]->busy()
            &&remaining>leading.config().toleranceMm) {
            transfer.syncPausedAxis=ahead;
            leading.pauseHeldMotion();
        }
        float adjustment=constrain(geometry.heldSyncGainPwmPerMm*error,-geometry.maxHeldSyncPwm,geometry.maxHeldSyncPwm);
        motor1.synchronization(-adjustment,dir);motor2.synchronization(adjustment,-dir);
    }
    return true;
}
void coordinate() {
    if (!initialized) return;
    if (transfer.active&&!superviseHeldTransfer()) return;
    if (!busy()) return;
    for (int i=0;i<2;++i) {
        if (axes[i]->hasFault()) { fault(axes[i]->message(),i);return; }
        if (process!=ProcessState::Positioning) continue; // HOME ignora CONTACTO y permite salir de END.
        if (events[i].reason==Reason::Contact) {
            auto& e=events[i];
            if (contact[i].confirmed) e.confirmedContact=true;
            if (e.confirmedContact&&!axes[i]->contactPressed()) {
                if (!e.lossPending) { e.lossPending=true;e.lossAt=millis(); }
                if (uint32_t(millis()-e.lossAt)>=geometry.contactLossConfirmMs) {
                    fault("CONTACTO confirmado perdido mientras el otro eje aproxima",i);return;
                }
            } else e.lossPending=false;
            continue;
        }
        if (events[i].reason!=Reason::None) continue;
        if (axes[i]->contactPressed()||axes[i]->state()==AxisState::ContactLimit) recordSensorStop(i,Reason::Contact);
        else if (axes[i]->state()==AxisState::EndLimit||
            (axes[i]->endPressed()&&(axes[i]->pwm()>0||axes[i]->target()>startPosition[i]))) recordSensorStop(i,Reason::End);
        else if (axes[i]->state()==AxisState::HomeLimit) { fault("HOME impide alcanzar objetivo",i);return; }
    }
    if (process==ProcessState::Translating) {
        for (int i=0;i<2;++i) {
            auto state=axes[i]->state();
            if (state==AxisState::HomeLimit||state==AxisState::EndLimit||state==AxisState::ContactLimit) {
                fault("Sensor impide completar traslado sujeto",i);return;
            }
        }
        if (motor1.state()==AxisState::Reached&&motor2.state()==AxisState::Reached) { process=ProcessState::Settling;settlingAt=millis(); }
        return;
    }
    if (process==ProcessState::Homing&&motor1.state()==AxisState::Ready&&motor2.state()==AxisState::Ready) {
        process=ProcessState::Ready;setMessage("LISTO: HOME completo; D,separacion_mm para aproximar");notifyEvent();printTravelRange();requestHelp();
    } else if (process==ProcessState::Positioning) {
        bool done=true;
        for (int i=0;i<2;++i) done&=events[i].reason!=Reason::None||axes[i]->state()==AxisState::Reached;
        if (done) { process=ProcessState::Settling;settlingAt=millis(); }
    }
}
void serviceSettling() {
    if (process!=ProcessState::Settling||uint32_t(millis()-settlingAt)<geometry.contactSettlingMs) return;
    if (transfer.active&&!superviseHeldTransfer(true)) return;
    if (bothContactsConfirmed()) {
        if (!transfer.active) { transfer=HeldTransfer{};transfer.active=true;transfer.referenceGap=faceGap(geometry,motor1.position(),motor2.position()); }
        process=ProcessState::Held;setMessage("SUJETA: motores detenidos; C centrar EST, T,delta trasladar, H abrir");
    } else {
        process=hasSensorStop()?ProcessState::SensorStopped:ProcessState::Ready;
        setMessage(hasSensorStop()?"FIN: sensor/objetivo; sujecion incompleta, revisar Q; H permite abrir":"LISTO: posicion alcanzada; PWM=0");
    }
    notifyEvent();printSeparation();printTravelRange();
}
bool printStopAxis(int i) {
    char text[180];
    const auto& s=faultSnapshot;
    if (s.transferTimedOut) {
        snprintf(text,sizeof(text),"AL VENCER M%d: inicio=%.3f x=%.3f objetivo=%.3f falta=%+.3f mm; PWM=%.2f%% CONTACTO=%d cuentas=%ld",
            i+1,transfer.start[i],s.x[i],s.target[i],s.target[i]-s.x[i],
            s.pwm[i],s.contact[i],static_cast<long>(s.encoder[i].count));
    } else {
        snprintf(text,sizeof(text),"ANTES DEL FALLO M%d: x=%.3f r=%.3f PWM=%.2f ayuda_sync=%.2f",
            i+1,s.x[i],s.target[i],s.pwm[i],s.syncBoost[i]);
    }
    return printReply(text);
}
bool printTimeoutGap() {
    char text[144];
    const float gap=faceGap(geometry,faultSnapshot.x[0],faultSnapshot.x[1]);
    snprintf(text,sizeof(text),"Separacion REF: inicio=%.3f al vencer=%.3f cambio=%+.3f mm",
        transfer.referenceGap,gap,gap-transfer.referenceGap);
    return printReply(text);
}
void serviceReport() {
    if (reportStep<0||uint32_t(millis()-reportAt)<REPORT_INTERVAL_MS) return;
    bool queued=false;char text[180];
    // Q: estado, tres motores y separacion. E mantiene el diagnostico ampliado.
    if (!encoderReport&&faultSnapshot.transferTimedOut) {
        switch (reportStep) {
        case 0:queued=printReply(processMessage);break;
        case 1:case 2:queued=printStopAxis(reportStep-1);break;
        case 3:queued=printTimeoutGap();break;
        case 4:queued=Panel::report();break;
        default:reportStep=-1;return;
        }
    } else if (!encoderReport) {
        switch (reportStep) {
        case 0:
            snprintf(text,sizeof(text),"SEP-r22 | M1/M2: %s",processMessage);
            queued=printReply(text);break;
        case 1:case 2: {
            int i=reportStep-1;const auto& a=*axes[i];
            snprintf(text,sizeof(text),"M%d: x=%.3f objetivo=%.3f mm PWM=%.1f%% HOME=%d END=%d CONTACTO=%d",
                i+1,a.position(),a.target(),a.pwm(),a.homePressed(),a.endPressed(),a.contactPressed());
            queued=printReply(text);break;
        }
        case 3:queued=Motor3::reportEssential();break;
        case 4:queued=printSeparation();break;
        case 5:queued=printTravelRange();break;
        case 6:queued=Panel::report();break;
        default:reportStep=-1;return;
        }
    } else switch(reportStep) {
    case 0:queued=printReply(processMessage);break;
    case 1:queued=printReply("IZQUIERDA (MOTOR 1):");break;
    case 2:queued=encoderReport?printEncoder(motor1):printStatus(motor1);break;
    case 3:queued=printReply(motor1.message());break;
    case 4:queued=printReply("DERECHA (MOTOR 2):");break;
    case 5:queued=encoderReport?printEncoder(motor2):printStatus(motor2);break;
    case 6:queued=printReply(motor2.message());break;
    case 7:queued=printSeparation();break;
    case 8:case 9: {
        int i=reportStep-8;const auto& e=events[i];
        if (e.reason==Reason::None) { ++reportStep;return; }
        snprintf(text,sizeof(text),"PARADA M%d por %s: t=%lu ms; x1=%.3f x2=%.3f; separacion_REF=%.3f mm",
            i+1,reasonName(e.reason),static_cast<unsigned long>(e.at),e.x[0],e.x[1],faceGap(geometry,e.x[0],e.x[1]));
        queued=printReply(text);break;
    }
    case 10:
        if (!hasSensorStop()) { reportStep=12;return; }
        queued=printReply("Registro en primera deteccion de sensor; no confirma fuerza ni ancho. No reactiva por rebote");break;
    case 11:
        if (!hasSensorStop()) { reportStep=12;return; }
        queued=printReply(contactCorrectionAllowed()?"Correccion fija solicitada aplicada; ancho real pendiente de calibracion":
            "Correccion de contactos pendiente: requiere HOME valido y ambos CONTACTO confirmados");break;
    case 12:case 13: {
        if (!faultSnapshot.valid) { reportStep=14;return; }
        queued=printStopAxis(reportStep-12);break;
    }
    case 14:
        snprintf(text,sizeof(text),"Firmware SEP-r22; HOME total=%.3f mm; izq_centro=%.3f der_centro=%.3f mm; contactos=%d/%d; centro estimado",
            geometry.homeFaceGapMm(),geometry.leftHomeToCenterMm,geometry.rightHomeToCenterMm,contact[0].confirmed,contact[1].confirmed);
        queued=printReply(text);break;
    case 15:queued=printReply("DIAGNOSTICO MOTOR 1 (estadisticas desde arranque):");break;
    case 16:queued=printPins(motor1);break;
    case 17:queued=printEncoderDiagnostic(motor1.encoderDiagnostic(),config1);break;
    case 18:queued=printReply("DIAGNOSTICO MOTOR 2 (estadisticas desde arranque):");break;
    case 19:queued=printPins(motor2);break;
    case 20:queued=printEncoderDiagnostic(motor2.encoderDiagnostic(),config2);break;
    case 21:queued=Motor3::reportEssential();break;
    case 22:queued=Motor3::reportPins();break;
    case 23:queued=Motor3::reportEncoder();break;
    case 24:queued=printTravelRange();break;
    case 25:queued=Panel::report();break;
    case 26:
        if (!faultSnapshot.valid) { reportStep=-1;return; }
        queued=printReply(faultSnapshot.transferTimedOut ? "ENCODERS AL VENCER EL PLAZO (M1 y M2):"
            : "ENCODERS AL DETECTAR FALLO (M1 y M2, respectivamente):");break;
    case 27:queued=printEncoderDiagnostic(faultSnapshot.encoder[0],config1);break;
    case 28:queued=printEncoderDiagnostic(faultSnapshot.encoder[1],config2);break;
    default:reportStep=-1;return;
    }
    if (queued) { ++reportStep;reportAt=millis(); }
}
void setup() {
    digitalWrite(config1.pwmPin,LOW);pinMode(config1.pwmPin,OUTPUT);
    digitalWrite(config2.pwmPin,LOW);pinMode(config2.pwmPin,OUTPUT);
    digitalWrite(config3.pwmPin,LOW);pinMode(config3.pwmPin,OUTPUT);
    Panel::begin(); // Contactor OFF y RGB antes del HOME automatico.
    Serial.begin(SERIAL_BAUD);
    if (!validPins()||!isfinite(geometry.leftHomeToCenterMm)||geometry.leftHomeToCenterMm<=0||
        !isfinite(geometry.rightHomeToCenterMm)||geometry.rightHomeToCenterMm<=0||
        !isfinite(geometry.contactOffsetMm)||!isfinite(geometry.leftContactShare)||geometry.leftContactShare<0||geometry.leftContactShare>1||
        !isfinite(geometry.maxHeldGapChangeMm)||geometry.maxHeldGapChangeMm<=0||
        !isfinite(geometry.heldSyncPauseMm)||geometry.heldSyncPauseMm<geometry.maxHeldGapChangeMm||
        !isfinite(geometry.heldSyncGainPwmPerMm)||geometry.heldSyncGainPwmPerMm<0||
        !isfinite(geometry.maxHeldSyncPwm)||geometry.maxHeldSyncPwm<0||
        !isfinite(geometry.centerToleranceMm)||geometry.centerToleranceMm<0||
        geometry.heldTimeoutMs<=geometry.contactSettlingMs||
        geometry.heldTimeoutMs>min(config1.moveTimeoutMs,config2.moveTimeoutMs)||
        geometry.contactLossConfirmMs==0||geometry.contactRecoverTimeoutMs<geometry.contactLossConfirmMs) { fault("Pines/geometria invalidos");return; }
    bool a=motor1.begin(),b=motor2.begin(),c=Motor3::begin();initialized=a&&b&&c;
    if (!initialized) { fault("Inicializacion fallida; revisar configuracion");return; }
    printReply("ARRANQUE: HOME automatico sin objeto. S/X no sustituye parada fisica");startHome(true);
}
void loop() {
    Panel::update(); // Tambien funciona si falla la inicializacion de motores.
    Command c;
    if (commands.poll(c)) switch(c.type) {
    case CommandType::ContactorOn:
        if (initialized&&process!=ProcessState::Fault) Panel::on();
        else printReply("ON rechazado: corregir fallo/inicializacion");
        break;
    case CommandType::ContactorOff:Panel::off();printReply("Orden OFF recibida");break;
    case CommandType::Home:startHome();break;
    case CommandType::Separation:startSeparation(c.value);break;
    case CommandType::Center:startTransfer(0,true);break;
    case CommandType::Translate:startTransfer(c.value,false);break;
    case CommandType::Move3:
    case CommandType::Home3:
    case CommandType::End3:
        if (!initialized) printReply("Inicializacion incompleta; corregir y reiniciar");
        else if (busy()||process==ProcessState::Settling) printReply("M1/M2 ocupados; esperar o S para cancelar antes de M3");
        else if (c.type==CommandType::Move3) Motor3::move(c.value,c.direction);
        else Motor3::seekLimit(c.type==CommandType::Home3?-1:1);
        break;
    case CommandType::Stop:
        stopAll();
        if (process!=ProcessState::Fault) { process=ProcessState::Stopped;setMessage("STOP: tres PWM=0; H/L antes de D; M3 acepta nueva orden relativa sin HOME"); }
        notifyEvent();break;
    case CommandType::Status:requestReport();break;
    case CommandType::Encoder:requestReport(true);break;
    case CommandType::Help:requestHelp();break;
    case CommandType::Invalid:printReply("Comando invalido: ? ayuda; ejemplo D,220");break;
    }
    serviceContacts();
    if (transfer.active) superviseHeldTransfer();
    if (busy()) {
        uint32_t us=micros();
        uint32_t maxGap=config1.maxControlGapUs<config2.maxControlGapUs?config1.maxControlGapUs:config2.maxControlGapUs;
        if (uint32_t(us-previousLoopUs)>maxGap) fault("Bucle de control excedido");
        previousLoopUs=us;coordinate();
        for (int i=0;i<2;++i) {
            if (!busy()||(transfer.active&&transfer.paused)) break;
            const bool waitForOther=transfer.active&&transfer.syncPausedAxis==i;
            axes[i]->update(micros(),millis(),waitForOther);
            coordinate();
        }
    }
    if (initialized) Motor3::update();
    serviceSettling();serviceReport();serviceHelp();serviceTelemetry();
}
