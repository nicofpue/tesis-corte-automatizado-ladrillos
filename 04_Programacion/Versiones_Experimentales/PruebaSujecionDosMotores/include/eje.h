#pragma once
#include "encoder.h"
#include "motor.h"
#include "finales.h"
#include "pid_posicion.h"

enum class AxisState {
    NotHomed, Ready, Homing, ConfirmHome, Moving, Reached, Stopped,
    EndLimit, ContactLimit, HomeLimit, Fault
};

// Una instancia por motor: encoder y PID independientes, con tres finales.
class Axis {
public:
    explicit Axis(const AxisConfig& config);
    Axis(const Axis&) = delete;
    Axis& operator=(const Axis&) = delete;
    bool begin();
    void update(uint32_t nowUs, uint32_t nowMs, bool waitForOtherAxis = false);
    bool home();
    bool moveTo(float millimeters);
    bool approachTo(float millimeters);
    bool moveHeldTo(float millimeters);
    void pauseHeldMotion(); // PWM=0 conservando estado, objetivo y timeout.
    void stop();
    void synchronization(float boost, int8_t direction) { syncBoost_=boost; syncDirection_=direction; }
    float syncBoost() const { return syncBoost_; }
    float faultPwm() const { return faultPwm_; }
    float faultSyncBoost() const { return faultSyncBoost_; }
    EncoderDiagnostic encoderDiagnostic() const { return encoder_.diagnostic(); }
    EncoderDiagnostic faultEncoderDiagnostic() const { return faultEncoder_; }
    const AxisConfig& config() const { return cfg_; }
    void haltContact() { contactLimit(); }
    void haltEnd() { endLimit(); }
    float position() const { return encoder_.position(); }
    int32_t counts() const { return encoder_.count(); }
    float target() const { return target_; }
    float pwm() const { return motor_.appliedPwm(); }
    bool isHomed() const { return homed_; }
    bool homePressed() const { return limits_.home(); }
    bool endPressed() const { return limits_.end(); }
    bool contactPressed() const { return limits_.contact(); }
    bool busy() const;
    bool hasFault() const { return state_ == AxisState::Fault; }
    AxisState state() const { return state_; }
    const char* stateName() const;
    const char* message() const { return message_; }
private:
    bool validConfig() const;
    void fail(const char* message);
    void confirmHome(uint32_t nowMs);
    void homeLimit();
    void endLimit();
    void contactLimit();
    AxisConfig cfg_;
    Encoder encoder_;
    Motor motor_;
    LimitSwitches limits_;
    PositionPid pid_;
    AxisState state_ = AxisState::NotHomed;
    bool initialized_ = false, homed_ = false;
    bool settling_ = false;
    bool heldMove_ = false; // CONTACTO debe permanecer activo durante traslado.
    bool fixedClosing_ = false; // Cierre de D con PWM fijo hasta sensor u objetivo.
    float movePwmLimit_ = 0;
    float faultPwm_ = 0;
    float faultSyncBoost_ = 0;
    EncoderDiagnostic faultEncoder_;
    float syncBoost_ = 0;
    int8_t syncDirection_ = 0;
    int32_t lastFeedbackCounts_ = 0;
    uint32_t lastFeedbackMs_ = 0;
    bool confirmHomeActive_ = false;
    float target_ = 0;
    float homeStartPosition_ = 0;
    uint32_t confirmEnteredMs_ = 0;
    uint32_t previousControlUs_ = 0, moveStartMs_ = 0, confirmStartMs_ = 0, settleStartMs_ = 0;
    const char* message_ = "Ejecutar H/L para referenciar";
};
