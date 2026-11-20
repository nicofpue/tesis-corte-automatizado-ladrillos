#pragma once
#include <stdint.h>

// Eje independiente: desplazamiento PID y busqueda de finales a PWM fijo.
namespace Motor3 {
bool begin();
bool move(float distanceMm, int8_t direction);
bool seekLimit(int8_t direction);
bool busy();
void stop();
void update();
bool reportEssential();
bool reportPins();
bool reportEncoder();
}
