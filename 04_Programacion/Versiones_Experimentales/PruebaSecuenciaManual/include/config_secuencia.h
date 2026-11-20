#pragma once
#include <stdint.h>

namespace SecuenciaConfig {
#if defined(BANCO_CONTACTOR_SIN_SIERRA) && BANCO_CONTACTOR_SIN_SIERRA
constexpr bool BANCO_SIN_SIERRA = true;
#else
constexpr bool BANCO_SIN_SIERRA = false;
#endif
constexpr float BANCO_COMPRESION_MM = 10.0f;
constexpr float BANCO_COMPRESION_PWM = 40.0f;
constexpr uint32_t BANCO_COMPRESION_MAX_MS = 5000;
constexpr float BANCO_CAMBIO_SEPARACION_MM = 5.0f;
constexpr float BANCO_AJUSTE_CENTRADO_M1_MM = 1.0f; // Negativo hacia HOME, positivo hacia END. Antes -1.0 (+2 mm hacia END).
constexpr float BANCO_AJUSTE_CENTRADO_M2_MM = -1.0f; // Positivo hacia END, negativo hacia HOME. Antes 1.0 (2 mm mas hacia HOME).
constexpr uint32_t BANCO_PERDIDA_CONTACTO_MS = 2000; // Inactividad continua por mordaza.
// En el perfil normal, false simula el contactor. El perfil de banco acciona la salida.
constexpr bool CONTACTOR_REAL_PERFIL_NORMAL = true;
constexpr bool HABILITAR_CONTACTOR_REAL = BANCO_SIN_SIERRA || CONTACTOR_REAL_PERFIL_NORMAL;
constexpr uint32_t ARRANQUE_SIMULADO_MS = 2000;
constexpr uint32_t ARRANQUE_REAL_MS = 2000; // Medido en el banco. 0 solicita confirmacion con N.
constexpr uint32_t HOME_ARRANQUE_PERDIDA_MS = 1000; // HOME inactivo continuamente antes de bloquear.
constexpr uint32_t HOME_ARRANQUE_ESTABILIZACION_MAX_MS = 1200; // Limite total si la entrada fluctua.
constexpr uint32_t ESTABILIZACION_FINAL_CENTRADO_MS = 100; // Espera adicional con ambas mordazas en objetivo.
// Margenes de deriva una vez completado el centrado, en mm.
constexpr float DERIVA_CENTRO_MM = 0.50f; // Desplazamiento del centro estimado de la pieza.
constexpr float DERIVA_SEPARACION_MM = 0.80f;
constexpr float DERIVA_MARGEN_VIBRACION_MM = 2.0f; // Margen adicional solo despues de centrar.
constexpr float DERIVA_BANDA_TRANSITORIA_MM = 0.8f; // Una cuenta del encoder de una mordaza.
constexpr uint32_t DERIVA_CONFIRMACION_MS = 300; // Solo para un exceso marginal.
constexpr uint32_t PARADA_REAL_VERIFICADA_MS = 20000; // 0 exige confirmar la liberacion con N.
constexpr uint32_t APERTURA_TRAS_PARADA_MS = 20000; // Desde CONTACTOR OFF al terminar el ciclo.
constexpr uint32_t ESPERA_EMERGENCIA_MS = 20000; // Desde la activacion, antes de autorizar HOME.
constexpr uint32_t PARADA_MS = 25000; // Recuperacion al encender y despues de STOP.
constexpr uint32_t CONFIRMACION_CON_SIERRA_MS = 20000; // Para esperar N antes de corte o retorno.
constexpr uint32_t CONFIRMACION_REINICIO_MS = 5000; // Dos START o dos I en este plazo.

static_assert(APERTURA_TRAS_PARADA_MS > 0 && APERTURA_TRAS_PARADA_MS >= PARADA_REAL_VERIFICADA_MS,
              "La espera de apertura debe cubrir la parada verificada");
static_assert(ESPERA_EMERGENCIA_MS > 0 && ESPERA_EMERGENCIA_MS >= PARADA_REAL_VERIFICADA_MS,
              "La espera de emergencia debe cubrir la parada verificada");
static_assert(PARADA_MS >= APERTURA_TRAS_PARADA_MS,
              "La recuperacion debe esperar al menos el tiempo de apertura");
static_assert(HOME_ARRANQUE_PERDIDA_MS > 0 && HOME_ARRANQUE_ESTABILIZACION_MAX_MS >= HOME_ARRANQUE_PERDIDA_MS,
              "La ventana de HOME debe cubrir la confirmacion de perdida");
static_assert(CONFIRMACION_CON_SIERRA_MS > 0, "La espera manual debe tener limite");
}
