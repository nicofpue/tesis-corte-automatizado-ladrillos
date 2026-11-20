#pragma once
#include <stdint.h>

// Eje independiente: desplazamiento PID y busqueda de finales con rampa.
namespace Motor3 {
bool begin();
bool move(float distanceMm, int8_t direction);
bool seekLimit(int8_t direction);
bool busy();
bool hasFault();
const char* message();
bool isHomed();
bool homePressed();
bool endPressed();
void stop();
void update();
bool reportEssential();
bool reportFault();
bool reportPins();
bool reportEncoder();
}
