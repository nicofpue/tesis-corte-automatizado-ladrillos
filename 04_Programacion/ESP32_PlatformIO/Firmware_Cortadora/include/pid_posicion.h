#pragma once
// =====================================================================
// pid_posicion.h - Controlador PID de posicion (mm -> %PWM)
// ---------------------------------------------------------------------
// Ganancias calculadas por ubicacion de polos sobre el modelo identificado
// X/U = Kv / (s (tau s + 1)) con zona muerta (valores en config_eje.h).
// Derivada sobre la medida (filtrada) y anti-windup por integracion condicional.
// =====================================================================
#include "config_eje.h"

class PositionPid {
public:
    explicit PositionPid(const PidConfig& config) : cfg_(config) {}
    void reset(float position);
    float update(float target, float position, float dt, float outputLimit);
    float velocity() const { return velocity_; }
    float p() const { return p_; }
    float i() const { return i_; }
    float d() const { return d_; }
private:
    PidConfig cfg_;
    float integral_ = 0, previousPosition_ = 0, velocity_ = 0;
    float p_ = 0, i_ = 0, d_ = 0;
};
