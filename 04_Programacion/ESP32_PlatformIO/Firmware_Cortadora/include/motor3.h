#pragma once
// =====================================================================
// motor3.h - Eje de transporte (M3)
// ---------------------------------------------------------------------
// Lleva la base con la pieza hacia la sierra (END) y de regreso (HOME).
// En el ciclo trabaja en lazo abierto: seekLimit() con rampa de 0 a 65 % en
// 500 ms hasta el final. move() (PID relativo) queda para pruebas manuales.
// =====================================================================
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
