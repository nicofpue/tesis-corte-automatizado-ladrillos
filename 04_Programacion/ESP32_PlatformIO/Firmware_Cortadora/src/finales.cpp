// =====================================================================
// finales.cpp - Lectura de finales y CONTACTO
// ---------------------------------------------------------------------
// =====================================================================
#include "finales.h"
// Configura las tres entradas del eje.
void LimitSwitches::begin() {
    pinMode(cfg_.homePin, cfg_.limitInputMode);
    pinMode(cfg_.endPin, cfg_.limitInputMode);
    pinMode(cfg_.contactPin, cfg_.limitInputMode);
}
// Lectura directa (true = sensor activo). El antirrebote lo hace quien la usa.
bool LimitSwitches::home() const { return digitalRead(cfg_.homePin) == cfg_.limitActiveLevel; }
bool LimitSwitches::end() const { return digitalRead(cfg_.endPin) == cfg_.limitActiveLevel; }
bool LimitSwitches::contact() const { return digitalRead(cfg_.contactPin) == cfg_.limitActiveLevel; }
