#pragma once
#include <Arduino.h>

// Panel: resistencias externas a 3.3 V; pulsadores/contactos hacia GND.
namespace PanelConfig {
constexpr uint8_t contactorPin=38, contactorActive=HIGH;
constexpr uint8_t startPin=16, stopPin=21;
// false: START NO / STOP NC. true: START NC / STOP NO.
constexpr bool startNormallyClosed=false;
constexpr bool stopNormallyClosed=!startNormallyClosed;
constexpr uint8_t startActive=startNormallyClosed ? HIGH : LOW;
constexpr uint8_t stopActive=stopNormallyClosed ? HIGH : LOW;
constexpr uint8_t emergencyPin=43, emergencyActive=HIGH; // TX como GPIO; NC.
constexpr uint32_t debounceMs=30;
constexpr uint8_t rgbPin=48, brightness=32;
// Cubierta GPIO47 sin configurar ni leer en esta prueba.
// EMERGENCIA, START y STOP son SOLO avisos; el corte es externo.
}
