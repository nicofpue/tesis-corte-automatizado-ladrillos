#pragma once
// =====================================================================
// finales.h - Finales de carrera y sensor de CONTACTO de un eje
// ---------------------------------------------------------------------
// Entradas con pull-up interno, activas en bajo (el sensor cierra a GND).
// HOME = mordaza abierta (origen), END = fin de recorrido,
// CONTACTO = la mordaza toco la pieza (sensor en el acople con resorte).
// =====================================================================
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
