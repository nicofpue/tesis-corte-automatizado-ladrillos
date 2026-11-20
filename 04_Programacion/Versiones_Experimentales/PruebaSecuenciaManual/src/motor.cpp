#include "motor.h"
#include <esp_arduino_version.h>
#include <math.h>

bool Motor::begin() {
    pinMode(cfg_.pwmPin, OUTPUT);
    digitalWrite(cfg_.pwmPin, LOW);
    pinMode(cfg_.dirPin, OUTPUT);
    digitalWrite(cfg_.dirPin, cfg_.positiveDirLevel);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ready_ = ledcAttachChannel(cfg_.pwmPin, cfg_.pwmFrequency, cfg_.pwmBits, cfg_.pwmChannel);
#else
    ready_ = ledcSetup(cfg_.pwmChannel, cfg_.pwmFrequency, cfg_.pwmBits) > 0;
    if (ready_) ledcAttachPin(cfg_.pwmPin, cfg_.pwmChannel);
#endif
    stop();
    return ready_;
}
void Motor::writeDuty(float magnitude) {
    if (!ready_) return;
    const uint32_t duty = lroundf(magnitude * ((1UL << cfg_.pwmBits) - 1) / 100.0f);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(cfg_.pwmPin, duty);
#else
    ledcWrite(cfg_.pwmChannel, duty);
#endif
}
void Motor::stop() {
    if (applied_ != 0) stoppedAt_ = micros();
    writeDuty(0);
    applied_ = 0;
}
void Motor::drive(float command, uint32_t nowUs) {
    if (!ready_ || !isfinite(command) || command == 0) { stop(); return; }
    const float pidLimit = fmaxf(cfg_.maxPwm, cfg_.heldMaxPwm);
    const float closingLimit = fmaxf(pidLimit, fmaxf(cfg_.contactPwm, cfg_.compressionPwm));
    command = constrain(command, -pidLimit, closingLimit);
    const int direction = command > 0 ? 1 : -1;
    if (lastDirection_ != 0 && direction != lastDirection_) {
        const bool wasRunning = applied_ != 0;
        stop();
        if (wasRunning) stoppedAt_ = nowUs;
        // Sin delay(): los finales y el puerto serie siguen siendo atendidos.
        if (uint32_t(nowUs - stoppedAt_) < cfg_.directionPauseUs) return;
    }
    digitalWrite(cfg_.dirPin, direction > 0 ? cfg_.positiveDirLevel : !cfg_.positiveDirLevel);
    lastDirection_ = direction;
    applied_ = command;
    writeDuty(fabsf(command));
}
