#pragma once
namespace Panel {
enum class Indicador { Espera, Listo, Movimiento, Bloqueado };
void begin();
void update();
void on();
void off();
bool blocked();
bool emergencyActive();
bool nuevoStop();
bool nuevoStart(bool habilitado);
void indicar(Indicador estado);
bool report();
}
