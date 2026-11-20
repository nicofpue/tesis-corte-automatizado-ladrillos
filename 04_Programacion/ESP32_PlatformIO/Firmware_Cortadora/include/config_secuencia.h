#pragma once
// =====================================================================
// config_secuencia.h - Parametros del ciclo automatico
// ---------------------------------------------------------------------
// Orden de la sujecion, compresion, tolerancias del centrado y tiempos
// de seguridad de la sierra. Cambiar aqui para ajustar el ciclo.
// =====================================================================
#include <stdint.h>

// ============================================================
// PARAMETROS DEL CICLO AUTOMATICO
// Unidades: mm, ms y PWM en porcentaje.
// ============================================================
namespace SecuenciaConfig {

// --- Orden de la sujecion ---
// true : CONTACTO -> centrado con presion ligera -> compresion (apriete final).
// false: CONTACTO -> compresion -> centrado (orden de PruebaSecuenciaManual).
constexpr bool COMPRIMIR_DESPUES_DE_CENTRAR = true;

// --- Compresion posterior al CONTACTO de cada mordaza ---
constexpr float    COMPRESION_MM     = 10.0f;  // Avance adicional maximo tras CONTACTO.
constexpr float    COMPRESION_PWM    = 40.0f;  // PWM fijo durante la compresion.
constexpr uint32_t COMPRESION_MAX_MS = 5000;   // Tiempo maximo de la compresion.
constexpr uint32_t COMPRESION_FILTRO_CONTACTO_MS = 200; // Perdida de CONTACTO admitida (PWM=0 mientras falta).

// --- Centrado ---
// Desfase del centro de la pieza respecto al centro fijo. Positivo hacia la derecha:
// M1 avanza hacia END y M2 retrocede hacia HOME la misma distancia; la separacion no cambia.
constexpr float    DESFASE_CENTRO_MM = 1.0f;
constexpr float    TOLERANCIA_PARADA_CENTRADO_MM = 0.25f; // Cada mordaza apaga su motor al llegar a esta distancia.
constexpr float    TOLERANCIA_CENTRADO_MM        = 2.0f;  // Error final aceptado por mordaza.
constexpr float    TOLERANCIA_SEPARACION_MM      = 5.0f;  // Cambio admisible de separacion frente al objetivo.
constexpr uint32_t PLAZO_CENTRADO_MS             = 10000; // Tiempo maximo del centrado, pausas incluidas.

// --- Supervision de la pieza sujeta (centrado, apriete y corte) ---
constexpr uint32_t PERDIDA_CONTACTO_MS = 2000; // Inactividad continua admitida por mordaza.

// --- Sierra (contactor): tiempos fijos, sin confirmacion manual ---
constexpr uint32_t ARRANQUE_SIERRA_MS      = 2000;  // Tras contactor ON, antes de avanzar M3.
constexpr uint32_t APERTURA_TRAS_PARADA_MS = 20000; // Desde contactor OFF hasta abrir mordazas.

// --- Arranque, emergencia y recuperacion ---
constexpr uint32_t HOME_ARRANQUE_PERDIDA_MS = 1000;            // HOME inactivo continuo antes de bloquear.
constexpr uint32_t HOME_ARRANQUE_ESTABILIZACION_MAX_MS = 1200; // Limite total si la entrada fluctua.
constexpr uint32_t ESPERA_EMERGENCIA_MS = 20000;   // Desde la emergencia hasta autorizar HOME.
constexpr uint32_t PARADA_MS = 25000;              // Recuperacion al encender y despues de STOP.
constexpr uint32_t CONFIRMACION_REINICIO_MS = 5000; // Dos START o dos I en este plazo.

static_assert(ARRANQUE_SIERRA_MS > 0 && APERTURA_TRAS_PARADA_MS > 0, "Los tiempos de la sierra deben ser positivos");
static_assert(PARADA_MS >= APERTURA_TRAS_PARADA_MS && ESPERA_EMERGENCIA_MS >= APERTURA_TRAS_PARADA_MS,
              "La recuperacion debe esperar al menos el tiempo de apertura");
static_assert(HOME_ARRANQUE_PERDIDA_MS > 0 && HOME_ARRANQUE_ESTABILIZACION_MAX_MS >= HOME_ARRANQUE_PERDIDA_MS,
              "La ventana de HOME debe cubrir la confirmacion de perdida");
static_assert(TOLERANCIA_PARADA_CENTRADO_MM > 0 && TOLERANCIA_PARADA_CENTRADO_MM < TOLERANCIA_CENTRADO_MM,
              "La parada del motor debe ser mas fina que la aceptacion del centrado");
static_assert(COMPRESION_FILTRO_CONTACTO_MS < COMPRESION_MAX_MS, "El filtro debe ser menor que la compresion");
}
