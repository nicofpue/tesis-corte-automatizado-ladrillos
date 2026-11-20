#pragma once
#include "config_eje.h"

class LimitSwitches {
public:
    explicit LimitSwitches(const AxisConfig& cfg) : cfg_(cfg) {}
    void begin();
    bool home() const;
    bool end() const;
    bool contact() const;
private:
    AxisConfig cfg_;
};
