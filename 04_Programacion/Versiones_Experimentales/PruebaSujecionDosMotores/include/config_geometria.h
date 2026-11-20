#pragma once
#include <stdint.h>

// ============================================================
// GEOMETRIA RESPECTO AL CENTRO FIJO E INFORME FINAL
// ============================================================
struct GeometryConfig {
    // --- Medidas de cada cara al centro fijo, ambos HOME, sin compresion ---
    float leftHomeToCenterMm  = 126.0f; // Cara M1 al centro.
    float rightHomeToCenterMm = 130.0f; // Centro a cara M2.
    float homeFaceGapMm() const { return leftHomeToCenterMm + rightHomeToCenterMm; }

    // --- Espera con ambos ejes detenidos ---
    uint32_t contactSettlingMs = 300;

    // --- Desfase TOTAL calibrado, solo con AMBOS contactos confirmados ---
    // Misma sujecion: caras medidas 246.000 mm; referencia de carros 211.656 mm.
    // Cambio relativo al estado HOME: 246.000 - 211.656 = +34.344 mm.
    // No se conoce el reparto individual; no sumar otras correcciones de resortes.
    float contactOffsetMm = 34.344f; // Se SUMA una sola vez a D0-x1-x2.

    // --- Centro ESTIMADO: reparto supuesto del desfase total ---
    float leftContactShare = 0.5f; // 0..1; 0.5 supone 17.172 mm por lado, no medidos.

    // --- Traslado con pieza sujeta: protecciones propias, NO para aproximacion D ---
    float maxHeldGapChangeMm = 0.5f; // Cambio maximo de separacion de carros desde inicio.
    float heldSyncPauseMm = 1.0f; // En C/T, pausa el eje adelantado hasta recuperar sincronismo.
    float heldSyncGainPwmPerMm = 5.0f; // Ajuste continuo entre ejes; inicial por calibrar.
    float maxHeldSyncPwm = 5.0f; // Correccion maxima, respetando minMovePwm y maxPwm.
    float centerToleranceMm = 0.15f;
    uint32_t heldTimeoutMs = 10000; // Plazo total de C/T, incluida la estabilizacion.
    uint32_t contactLossConfirmMs = 30;
    uint32_t contactRecoverTimeoutMs = 2000;

};

inline GeometryConfig geometryConfig() {
    return GeometryConfig();
}

// ============================================================
// COMUNICACION SERIE
// ============================================================
constexpr uint32_t SERIAL_BAUD        = 115200;
constexpr uint32_t REPORT_INTERVAL_MS = 50;
constexpr bool AUTOMATIC_FULL_REPORTS = false; // Q: esencial; E: diagnostico completo.
