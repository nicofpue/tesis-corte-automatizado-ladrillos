#include "pid_posicion.h"
#include <math.h>

void PositionPid::reset(float position) {
    integral_ = velocity_ = p_ = i_ = d_ = 0;
    previousPosition_ = position;
}

float PositionPid::update(float target, float position, float dt, float limit) {
    if (!(dt > 0) || !isfinite(dt)) return 0;
    const float error = target - position;
    const float rawVelocity = (position - previousPosition_) / dt;
    previousPosition_ = position;
    const float alpha = dt / (cfg_.derivativeFilterS + dt);
    velocity_ += alpha * (rawVelocity - velocity_);
    p_ = cfg_.kp * error;
    // Derivada sobre medida: evita golpe D al cambiar la referencia.
    // Para referencia constante equivale a Kd * derivada(error).
    d_ = -cfg_.kd * velocity_;
    const float candidate = cfg_.ki > 0
        ? constrain(integral_ + error * dt, -cfg_.integralLimit, cfg_.integralLimit)
        : 0.0f;
    const float trial = p_ + cfg_.ki * candidate + d_;
    // Anti-windup por integracion condicional: permite desaturar.
    if (!((trial > limit && error > 0) || (trial < -limit && error < 0))) {
        integral_ = candidate;
    }
    i_ = cfg_.ki * integral_;
    return constrain(p_ + i_ + d_, -limit, limit);
}
