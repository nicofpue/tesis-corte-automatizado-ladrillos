#pragma once
// =====================================================================
// eje.h - Control de una mordaza (M1 o M2)
// ---------------------------------------------------------------------
// Integra encoder, driver, finales y PID de un eje. Cada eje tiene su propia
// maquina de estados; el coordinador (main.cpp) le da ordenes y llama
// update() en cada vuelta de loop().
// =====================================================================
#include "encoder.h"
#include "motor.h"
#include "finales.h"
#include "pid_posicion.h"

// Estados de un eje:
//  NotHomed     sin referencia (al encender o despues de un fallo)
//  Ready        HOME confirmado; cuentas en cero
//  Homing       abriendo hacia HOME
//  ConfirmHome  HOME detectado; motor parado mientras se confirma (30 ms)
//  Moving       movimiento en curso (PID o PWM fijo)
//  Reached      objetivo alcanzado (o centrado detenido con CONTACTO activo)
//  Stopped      detenido por STOP
//  EndLimit / ContactLimit / HomeLimit: detenido por un sensor
//  Fault        fallo: motor apagado hasta reiniciar
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
    // Ordenes (devuelven false si no se pueden iniciar; ver message()):
    //  home()       abre hasta HOME y pone el encoder en cero.
    //  moveTo()     PID hasta una posicion absoluta (mm desde HOME).
    //  approachTo() cierre a PWM fijo (25 %) hasta CONTACTO u objetivo.
    //  compressTo() compresion tras CONTACTO a PWM fijo (40 %).
    //  moveHeldTo() traslado de centrado con la pieza sujeta: exige CONTACTO.
    bool home();
    bool moveTo(float millimeters);
    bool approachTo(float millimeters);
    bool compressTo(float millimeters);
    bool moveHeldTo(float millimeters);
    void pauseHeldMotion(); // PWM=0 conservando estado, objetivo y timeout.
    void stop();
    // Empuje de sincronismo durante el centrado (lo fija main.cpp para la mordaza rezagada).
    void synchronization(float boost, int8_t direction) { syncBoost_=boost; syncDirection_=direction; }
    float syncBoost() const { return syncBoost_; }
    float faultPwm() const { return faultPwm_; }
    float faultSyncBoost() const { return faultSyncBoost_; }
    EncoderDiagnostic encoderDiagnostic() const { return encoder_.diagnostic(); }
    EncoderDiagnostic faultEncoderDiagnostic() const { return faultEncoder_; }
    const AxisConfig& config() const { return cfg_; }
    void haltContact() { contactLimit(); }
    void haltEnd() { endLimit(); }
    // Lecturas del eje.
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
    bool stalledHeld() const { return stalledHeld_; } // Centrado detenido sin avance, con CONTACTO activo.
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
    bool compressing_ = false;
    bool stalledHeld_ = false; // Centrado terminado sin cuentas y con CONTACTO; lo evalua el coordinador.
    bool compressionContactLost_ = false; // CONTACTO ausente durante la compresion (filtro).
    uint32_t compressionContactLostMs_ = 0;
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
