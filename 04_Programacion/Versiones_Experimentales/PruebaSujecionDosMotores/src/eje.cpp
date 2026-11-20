#include "eje.h"
#include <math.h>

Axis::Axis(const AxisConfig& cfg)
    : cfg_(cfg), encoder_(cfg), motor_(cfg), limits_(cfg), pid_(cfg.pid) {}

bool Axis::validConfig() const {
    const float values[] = {cfg_.countsPerRev, cfg_.leadMmRev, cfg_.encoderToOutputRatio, cfg_.toleranceMm,
        cfg_.maxTravelMm, cfg_.maxPwm, cfg_.heldMaxPwm, cfg_.contactPwm, cfg_.homePwm, cfg_.deadzonePwm, cfg_.minMovePwm,
        cfg_.settledSpeedMmS, cfg_.travelMarginMm, cfg_.pid.kp, cfg_.pid.ki, cfg_.pid.kd,
        cfg_.pid.integralLimit, cfg_.pid.derivativeFilterS};
    for (float v : values) if (!isfinite(v) || v < 0) return false;
    const uint8_t pins[] = {cfg_.pwmPin,cfg_.dirPin,cfg_.encA,cfg_.encB,
        cfg_.homePin,cfg_.endPin,cfg_.contactPin};
    for (unsigned i=0; i<7; ++i) {
        if (pins[i] == 255) return false;
        for (unsigned j=i+1; j<7; ++j) if (pins[i]==pins[j]) return false;
    }
    const float lowerPidLimit = fminf(cfg_.maxPwm, cfg_.heldMaxPwm);
    return cfg_.countsPerRev > 0 && cfg_.leadMmRev > 0 && cfg_.encoderToOutputRatio > 0 && cfg_.maxTravelMm > 0 && cfg_.toleranceMm > 0
        && cfg_.maxPwm > 0 && cfg_.maxPwm <= 100 && cfg_.contactPwm <= 100 && cfg_.homePwm > 0
        && cfg_.heldMaxPwm > 0 && cfg_.heldMaxPwm <= 100
        && cfg_.homePwm <= cfg_.maxPwm && cfg_.controlPeriodUs > 0
        && cfg_.maxControlGapUs >= cfg_.controlPeriodUs
        && cfg_.homeTimeoutMs > cfg_.limitConfirmMs && cfg_.moveTimeoutMs > cfg_.settlingMs
        && cfg_.homeConfirmTimeoutMs > cfg_.limitConfirmMs && cfg_.noCountsTimeoutMs > 0
        && cfg_.closingNoCountsTimeoutMs > 0 && cfg_.closingNoCountsTimeoutMs <= cfg_.moveTimeoutMs
        && cfg_.pwmBits > 0 && cfg_.pwmBits <= 14 && cfg_.pwmChannel < 8
        && cfg_.pwmFrequency > 0 && (cfg_.encoderSign == 1 || cfg_.encoderSign == -1)
        && (!cfg_.compensateDeadzone || (cfg_.deadzonePwm < lowerPidLimit
            && cfg_.minMovePwm <= lowerPidLimit));
}
bool Axis::begin() {
    if (!validConfig()) { fail("Configuracion invalida"); return false; }
    limits_.begin();
    if (!motor_.begin()) { fail("Fallo al configurar PWM"); return false; }
    encoder_.begin();
    initialized_ = true;
    if (limits_.home() && limits_.end()) {
        fail("HOME y END activos a la vez"); return false;
    }
    return true;
}
bool Axis::busy() const {
    return state_ == AxisState::Homing || state_ == AxisState::ConfirmHome || state_ == AxisState::Moving;
}
void Axis::fail(const char* message) {
    heldMove_ = false;
    faultEncoder_=encoder_.diagnostic();
    faultPwm_ = motor_.appliedPwm(); // Conservar esfuerzo antes de retirar PWM.
    faultSyncBoost_ = syncBoost_;
    syncBoost_ = 0;
    motor_.stop();
    homed_ = false;
    state_ = AxisState::Fault;
    message_ = message;
}
void Axis::stop() {
    heldMove_ = false;
    syncBoost_ = 0;
    motor_.stop();
    if (state_ == AxisState::Fault) return; // Conservar el diagnostico del fallo.
    if (state_ == AxisState::Homing || state_ == AxisState::ConfirmHome) homed_ = false;
    pid_.reset(position());
    if (state_ != AxisState::Fault) state_ = AxisState::Stopped;
    message_ = "STOP; no reanuda automaticamente";
}
bool Axis::home() {
    heldMove_ = false;
    syncBoost_ = 0;
    if (!initialized_) { message_ = "Inicializacion incompleta; reiniciar"; return false; }
    motor_.stop();
    homed_ = false; // Un homing fallido nunca conserva una referencia anterior valida.
    if (limits_.home() && limits_.end()) {
        fail("HOME y END activos a la vez"); return false;
    }
    pid_.reset(position());
    homeStartPosition_ = position();
    moveStartMs_ = millis();
    state_ = AxisState::Homing;
    message_ = "Buscando HOME";
    return true;
}
bool Axis::moveTo(float mm) {
    if (!initialized_ || !homed_) { message_ = "Primero completar H"; return false; }
    if (busy() && state_ != AxisState::Moving) { message_ = "Homing en curso"; return false; }
    if (!isfinite(mm) || mm < 0 || mm > cfg_.maxTravelMm) {
        message_ = "Objetivo fuera de recorrido"; return false;
    }
    if (limits_.home() && limits_.end()) {
        fail("HOME y END activos a la vez"); return false;
    }
    motor_.stop();
    heldMove_ = false;
    fixedClosing_ = false;
    movePwmLimit_ = cfg_.maxPwm;
    syncBoost_ = 0;
    target_ = mm;
    pid_.reset(position());
    previousControlUs_ = micros();
    moveStartMs_ = millis();
    settling_ = false;
    lastFeedbackCounts_ = counts();
    lastFeedbackMs_ = millis();
    state_ = AxisState::Moving;
    message_ = "Control de posicion activo";
    return true;
}
bool Axis::approachTo(float mm) {
    if (!moveTo(mm)) return false;
    fixedClosing_ = cfg_.contactPwm > 0 && mm > position();
    return true;
}
bool Axis::moveHeldTo(float mm) {
    if (!contactPressed()) { message_ = "CONTACTO inactivo al iniciar traslado"; return false; }
    if (!moveTo(mm)) return false;
    movePwmLimit_ = cfg_.heldMaxPwm;
    heldMove_ = true;
    return true;
}
void Axis::pauseHeldMotion() {
    if (state_!=AxisState::Moving) return;
    motor_.stop();pid_.reset(position());previousControlUs_=micros();settling_=false;
}
void Axis::confirmHome(uint32_t nowMs) {
    motor_.stop(); // Se detiene con la primera lectura, ANTES del antirrebote.
    confirmStartMs_ = nowMs;
    confirmEnteredMs_ = nowMs;
    confirmHomeActive_ = true;
    state_ = AxisState::ConfirmHome;
    message_ = "Confirmando HOME; motor detenido";
}
void Axis::homeLimit() {
    motor_.stop();
    pid_.reset(position()); // Reinicia el PID, NO las cuentas del encoder.
    settling_ = false;
    state_ = AxisState::HomeLimit;
    message_ = "HOME: retorno bloqueado; encoder conservado. Solo H recalibra";
}
void Axis::endLimit() {
    motor_.stop();
    pid_.reset(position());
    state_ = AxisState::EndLimit;
    message_ = "END: se permite ordenar retorno hacia HOME";
}
void Axis::contactLimit() {
    motor_.stop();
    pid_.reset(position());
    state_ = AxisState::ContactLimit;
    message_ = "CONTACTO: se permite ordenar retorno hacia HOME";
}
void Axis::update(uint32_t nowUs, uint32_t nowMs, bool waitForOtherAxis) {
    if (!initialized_) return;
    const bool homeActive = limits_.home(), endActive = limits_.end();
    const bool contactActive = limits_.contact();
    // END y CONTACTO pueden estar activos juntos: ambos bloquean avance.
    if (homeActive && endActive) {
        fail("HOME y END activos a la vez"); return;
    }

    if (state_ == AxisState::ConfirmHome) {
        motor_.stop();
        // Un rebote reinicia la confirmacion sin reactivar el motor.
        // Solo se acepta cero tras 30 ms de lecturas activas consecutivas.
        if (uint32_t(nowMs - moveStartMs_) >= cfg_.homeTimeoutMs ||
            uint32_t(nowMs - confirmEnteredMs_) >= cfg_.homeConfirmTimeoutMs) {
            fail("Timeout: HOME no se estabiliza; revisar conexion"); return;
        }
        if (!homeActive) {
            confirmHomeActive_ = false;
            return;
        }
        if (!confirmHomeActive_) {
            confirmHomeActive_ = true;
            confirmStartMs_ = nowMs;
        }
        if (uint32_t(nowMs - confirmStartMs_) < cfg_.limitConfirmMs) return;
        // Este estado solo se alcanza desde un homing solicitado.
        // Un unico cero por H completado; HOME sostenido no vuelve a poner cero.
        encoder_.zero();
        homed_ = true;
        pid_.reset(0);
        target_ = 0;
        state_ = AxisState::Ready;
        message_ = "H completado; referencia cero establecida";
        return;
    }
    if (state_ == AxisState::Homing) {
        if (fabsf(position()-homeStartPosition_) > cfg_.maxTravelMm+cfg_.travelMarginMm) {
            fail("Recorrido HOME excedido"); return;
        }
        if (homeActive) { confirmHome(nowMs); return; }
        if (uint32_t(nowMs - moveStartMs_) >= cfg_.homeTimeoutMs) {
            fail("Timeout HOME"); return;
        }
        motor_.drive(-cfg_.homePwm, nowUs); // HOME explicito permite abrir con END/CONTACTO activos.
        return;
    }
    if (state_ != AxisState::Moving) return;

    // Revisar finales en CADA loop, no solo cada periodo PID.
    // HOME sostenido permite salir hacia positivo y NO modifica el encoder.
    if (homeActive && motor_.appliedPwm() < 0) {
        homeLimit(); return;
    }
    if (endActive && motor_.appliedPwm() > 0) { endLimit(); return; }
    if (!heldMove_ && contactActive && motor_.appliedPwm() > 0) { contactLimit(); return; }
    if (heldMove_ && !contactActive) { pauseHeldMotion();return; }
    if (uint32_t(nowMs - moveStartMs_) >= cfg_.moveTimeoutMs) { fail("Timeout movimiento"); return; }
    // Sin avance no implica encoder averiado: puede ser pausa de compresion.
    // El cierre permite 5 s configurables; apertura conserva su limite propio.
    const int32_t feedback = counts();
    if (feedback != lastFeedbackCounts_ || motor_.appliedPwm() == 0) {
        lastFeedbackCounts_ = feedback;
        lastFeedbackMs_ = nowMs;
    } else {
        const bool closing = motor_.appliedPwm() > 0;
        const uint32_t allowed = closing ? cfg_.closingNoCountsTimeoutMs : cfg_.noCountsTimeoutMs;
        if (uint32_t(nowMs - lastFeedbackMs_) >= allowed) {
            fail(closing ? "Pausa sin avance en cierre excedida: compresion/bloqueo" :
                           "Apertura sin cambio de cuentas con PWM activo");
            return;
        }
    }
    // Supervisar recorrido en cada loop, incluso entre periodos del PID.
    const float positionMm = position();
    if (positionMm < -cfg_.travelMarginMm || positionMm > cfg_.maxTravelMm+cfg_.travelMarginMm) {
        fail("Posicion fuera de recorrido configurado"); return;
    }
    if (waitForOtherAxis) { pauseHeldMotion(); return; }
    const float error = target_ - positionMm;
    // En cierre fijo cortar al llegar, incluso entre muestras del PID.
    if (fixedClosing_ && error <= cfg_.toleranceMm) motor_.stop();
    const uint32_t elapsed = nowUs - previousControlUs_;
    if (elapsed < cfg_.controlPeriodUs) return;
    if (elapsed > cfg_.maxControlGapUs) { fail("Periodo de control excedido"); return; }
    previousControlUs_ = nowUs;
    const float limit = movePwmLimit_ - (cfg_.compensateDeadzone ? cfg_.deadzonePwm : 0);
    float u = pid_.update(target_, positionMm, elapsed / 1000000.0f, limit);
    if (!isfinite(u)) { fail("Salida PID no finita"); return; }

    if ((fixedClosing_ ? error : fabsf(error)) <= cfg_.toleranceMm) {
        motor_.stop();
        if (fabsf(pid_.velocity()) > cfg_.settledSpeedMmS) { settling_ = false; return; }
        if (!settling_) { settling_ = true; settleStartMs_ = nowMs; }
        if (uint32_t(nowMs - settleStartMs_) >= cfg_.settlingMs) {
            state_ = AxisState::Reached;
            pid_.reset(positionMm);
            message_ = "Posicion alcanzada; PWM cero, sin mantenimiento activo";
        }
        return;
    }
    settling_ = false;
    if (fixedClosing_) u = cfg_.contactPwm;
    // Fuera de tolerancia, bloquear solo el sentido que empeora cada limite.
    if (u < 0 && homeActive) { homeLimit(); return; }
    if (u > 0 && endActive) { endLimit(); return; }
    if (!heldMove_ && u > 0 && contactActive) { contactLimit(); return; }
    if (u != 0 && !fixedClosing_) {
        float magnitude = fabsf(u);
        if (cfg_.compensateDeadzone)
            magnitude = fmaxf(magnitude + cfg_.deadzonePwm, cfg_.minMovePwm);
        // Ayudar gradualmente al rezagado, sin detener al adelantado.
        // No amplificar una correccion en sentido contrario al movimiento previsto.
        if (u * syncDirection_ > 0)
            magnitude=fmaxf(cfg_.compensateDeadzone?cfg_.minMovePwm:0.0f,magnitude+syncBoost_);
        u = copysignf(fminf(magnitude, movePwmLimit_), u);
    }
    motor_.drive(u, nowUs);
}
const char* Axis::stateName() const {
    switch (state_) {
    case AxisState::NotHomed: return "NOT_HOMED";
    case AxisState::Ready: return "READY";
    case AxisState::Homing: return "HOMING";
    case AxisState::ConfirmHome: return "CONFIRM_HOME";
    case AxisState::Moving: return "MOVING";
    case AxisState::Reached: return "POSITION_OK";
    case AxisState::Stopped: return "STOPPED";
    case AxisState::EndLimit: return "END_LIMIT";
    case AxisState::ContactLimit: return "CONTACT_LIMIT";
    case AxisState::HomeLimit: return "HOME_LIMIT";
    case AxisState::Fault: return "FAULT";
    }
    return "UNKNOWN";
}
