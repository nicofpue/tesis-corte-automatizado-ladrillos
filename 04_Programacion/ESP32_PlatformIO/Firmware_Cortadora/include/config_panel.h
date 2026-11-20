#pragma once
// =====================================================================
// config_panel.h - Pines y logica del panel
// ---------------------------------------------------------------------
// Contactor (GPIO38), START (GPIO16), STOP (GPIO21, deshabilitado),
// EMERGENCIA (GPIO43, normalmente cerrada) y LED RGB (GPIO48).
// =====================================================================
#include <Arduino.h>

// Panel: resistencias externas a 3.3 V; pulsadores/contactos hacia GND.
namespace PanelConfig {
constexpr uint8_t contactorPin=38, contactorActive=HIGH;
constexpr uint8_t startPin=16, stopPin=21;
constexpr bool physicalStopEnabled=false; // Temporal: usar S/X en el monitor.
// false: START NO / STOP NC. true: START NC / STOP NO.
constexpr bool startNormallyClosed=false;
constexpr bool stopNormallyClosed=!startNormallyClosed;
constexpr uint8_t startActive=startNormallyClosed ? HIGH : LOW;
constexpr uint8_t stopActive=stopNormallyClosed ? HIGH : LOW;
constexpr uint8_t emergencyPin=43, emergencyActive=HIGH; // TX como GPIO; NC.
constexpr uint32_t debounceMs=30;
constexpr uint8_t rgbPin=48, brightness=32;
constexpr uint32_t blinkHalfPeriodMs=500; // LED titilando: 0,5 s encendido / 0,5 s apagado.
// Cubierta GPIO47 sin configurar ni leer en esta prueba.
// EMERGENCIA es independiente de la habilitacion del STOP fisico.
}
