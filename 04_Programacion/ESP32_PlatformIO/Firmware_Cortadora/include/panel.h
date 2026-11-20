#pragma once
// =====================================================================
// panel.h - Panel del operador y contactor de la sierra
// ---------------------------------------------------------------------
// Entradas START, STOP y EMERGENCIA con antirrebote, salida del contactor
// y LED RGB de la placa (GPIO48) que indica el estado al operario.
// =====================================================================
namespace Panel {
enum class Color { Apagado, Verde, Amarillo, Azul, Rojo };
void begin();
void update();
void on();
void off();
bool blocked();
bool emergencyActive();
bool nuevoStop();
bool nuevoStart(bool habilitado);
void indicar(Color color, bool titila); // La EMERGENCIA activa tiene prioridad: rojo fijo.
bool report();
}
