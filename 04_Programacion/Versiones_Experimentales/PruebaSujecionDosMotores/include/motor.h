#pragma once
#include "config_eje.h"

class Motor {
public:
    explicit Motor(const AxisConfig& cfg) : cfg_(cfg) {}
    bool begin();
    void stop();
    void drive(float signedPercent, uint32_t nowUs);
    float appliedPwm() const { return applied_; } // Con signo funcional.
private:
    void writeDuty(float magnitude);
    AxisConfig cfg_;
    bool ready_ = false;
    int lastDirection_ = 0;
    uint32_t stoppedAt_ = 0;
    float applied_ = 0;
};
