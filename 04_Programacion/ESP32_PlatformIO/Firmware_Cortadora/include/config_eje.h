#pragma once
// =====================================================================
// config_eje.h - Parametros de los tres ejes
// ---------------------------------------------------------------------
// PID, PWM, sensores, escalas, recorridos y tiempos de supervision.
// M1 = mordaza izquierda, M2 = mordaza derecha, M3 = transporte.
// =====================================================================
#include <Arduino.h>
#include "config_secuencia.h"

// Ajustes compartidos en mordazaConfig(). Pines y recorridos en motor1Config() y motor2Config().
// Los demas valores de AxisConfig son comunes mientras no se sobrescriban abajo.
// Unidades: mm, segundos indicados con S, milisegundos con Ms, microsegundos con Us.
// PWM siempre en porcentaje. Positivo siempre hacia el centro.

// ============================================================
// PID: VALORES COMUNES POR DEFECTO
// ============================================================
struct PidConfig {
    float kp = 1.6468f; // %PWM/mm. Asignacion de polos con Kv=3.99 (mm/s)/%, tau=0.020 s.
    float ki = 2.6383f; // %PWM/(mm*s). Con Ki=Kd=0 se ejecuta solo P.
    float kd = 0.076832f; // %PWM*s/mm.
    float integralLimit     = 20.0f;  // Limite de integral(error*dt), mm*s.
    float derivativeFilterS = 0.03f;  // Filtro de derivada sobre medida.
};

// ============================================================
// EJE: PARAMETROS COMUNES POR DEFECTO
// ============================================================
// Tolerancias del centrado: ver TOLERANCIA_PARADA_CENTRADO_MM y TOLERANCIA_CENTRADO_MM en config_secuencia.h.

struct AxisConfig {
    // --- Pines: asignados individualmente en cada motor ---
    uint8_t pwmPin     = 255;
    uint8_t dirPin     = 255;
    uint8_t encA       = 255;
    uint8_t encB       = 255;
    uint8_t homePin    = 255;
    uint8_t endPin     = 255;
    uint8_t contactPin = 255;

    // --- Generacion de PWM ---
    uint8_t  pwmChannel   = 0;
    uint32_t pwmFrequency = 20000;
    uint8_t  pwmBits      = 10;

    // --- Sensores: a GND, activos LOW, pull-up interno ---
    uint8_t  limitActiveLevel     = LOW;
    uint8_t  limitInputMode       = INPUT_PULLUP;
    uint32_t limitConfirmMs       = 30;    // HOME estable, con motor detenido.
    uint32_t homeConfirmTimeoutMs = 2000;

    // --- Sentido y conversion del encoder ---
    uint8_t positiveDirLevel = LOW;      // Cierre, alejandose de HOME.
    int8_t  encoderSign      = -1;       // Posicion creciente al cerrar.
    float   countsPerRev     = 1000.0f;  // Cuentas x1: A/RISING.
    float   leadMmRev        = 8.0f;     // Avance por vuelta de salida.
    float   encoderToOutputRatio = 1.0f; // Vueltas del encoder por vuelta de salida.
    float   mmPerCount() const { return leadMmRev / (countsPerRev * encoderToOutputRatio); }

    // --- Control y confirmacion de posicion ---
    PidConfig pid;
    uint32_t controlPeriodUs  = 10000;
    uint32_t maxControlGapUs  = 100000; // Demora maxima del control.
    float    toleranceMm     = 0.15f;
    float    settledSpeedMmS = 0.5f;
    uint32_t settlingMs      = 300;

    // --- Recorrido y techo de salida ---
    float maxTravelMm    = 0.0f;  // Asignado en el bloque de cada motor.
    float travelMarginMm = 2.0f;  // Corte de emergencia; no amplia objetivos.
    float maxPwm         = 20.0f; // Techo del control por PID.
    float heldMaxPwm     = 25.0f; // Techo del PID durante el centrado con pieza sujeta.
    float compressionPwm = 0.0f; // PWM para avance posterior a CONTACTO (mordazas).
    float contactPwm     = 0.0f;  // PWM fijo de cierre con D; 0 conserva el PID.

    // --- Compensacion de zona muerta ---
    bool  compensateDeadzone = true;  // false: salida PID directa en %PWM.
    float deadzonePwm        = 5.55f;
    float minMovePwm         = 6.2f; // Valor anterior a los ajustes por conexiones defectuosas.

    // --- HOME, tiempos maximos y supervision ---
    float    homePwm          = 15.0f;
    float    endSeekPwm      = 0.0f;  // Solo M3: PWM fijo del comando E3.
    uint32_t homeTimeoutMs    = 15000;
    uint32_t moveTimeoutMs    = 120000; // Tiempo real maximo por orden.
    uint32_t noCountsTimeoutMs = 1000; // Apertura con PWM activo sin avance; no HOME.
    uint32_t closingNoCountsTimeoutMs = 5000; // Cierre: pausa de compresion permitida.
    uint32_t directionPauseUs = 20000; // PWM=0 antes de invertir DIR.
};


// ============================================================
// MORDAZAS: AJUSTES COMUNES Y PINES DE M1
// ============================================================
inline AxisConfig mordazaConfig() {
    AxisConfig c; // M1 y M2 usan los mismos valores de PidConfig.
    c.contactPwm = 25.0f;                                // PWM fijo de cierre hasta CONTACTO (como PruebaSujecionDosMotores).
    c.compressionPwm = SecuenciaConfig::COMPRESION_PWM;  // PWM de compresion tras CONTACTO.
    return c;
}

inline AxisConfig motor1Config() {
    AxisConfig c = mordazaConfig();

    // --- Pines ---
    c.pwmPin     = 40;
    c.dirPin     = 39;
    c.encA       = 9;
    c.encB       = 10;
    c.homePin    = 4;
    c.endPin     = 5;
    c.contactPin = 6;
    c.pwmChannel = 0;

    c.maxTravelMm = 68.0f;

    return c;
}

// ============================================================
// MOTOR 2: MORDAZA DERECHA
// ============================================================
inline AxisConfig motor2Config() {
    AxisConfig c = mordazaConfig();

    // --- Pines ---
    c.pwmPin     = 42;
    c.dirPin     = 41;
    c.encA       = 11;
    c.encB       = 12;
    c.homePin    = 17;
    c.endPin     = 18;
    c.contactPin = 8;
    c.pwmChannel = 1;

    c.maxTravelMm = 74.0f;

    return c;
}

// ============================================================
// MOTOR 3: PID RELATIVO Y BUSQUEDA DE FINALES POR COMANDO
// ============================================================
constexpr float M3_PWM_PERCENT = 65.0f;
constexpr uint32_t M3_SEEK_RAMP_MS = 500; // Rampa lineal desde 0 al buscar HOME o END.
constexpr float M3_HOME_APROX_MM = 10.0f; // Ultimo tramo del retorno referenciado.
constexpr float M3_HOME_APROX_PWM = M3_PWM_PERCENT; // Igual al PWM de retorno: sin tramo lento final.

inline AxisConfig motor3Config() {
    AxisConfig c;

    // --- Pines recuperados de CaracterizacionMotorLineal ---
    c.pwmPin     = 1;
    c.dirPin     = 2;
    c.encA       = 13;
    c.encB       = 14;
    c.homePin    = 7;
    c.endPin     = 15;
    c.contactPin = 255; // Sin CONTACTO; GPIO16 pertenece a START.
    c.pwmChannel = 2;
    c.limitInputMode = INPUT_PULLUP;
    c.limitActiveLevel = LOW;

    // --- Sentido y escala: 8/(1000*10) = 0.0008 mm/cuenta (encoder en el motor) ---
    c.positiveDirLevel = HIGH;
    c.encoderSign = 1; // Conserva el signo de avance tras invertir DIR.
    c.countsPerRev = 1000.0f;
    c.leadMmRev = 8.0f;
    c.encoderToOutputRatio = 10.0f;

    // --- PID de M3: solo para desplazamientos relativos; el ciclo usa H3/E3 con PWM fijo ---
    c.pid.kp = 0.60f;
    c.pid.ki = 0.00f;
    c.pid.kd = 0.00f;
    c.toleranceMm = 0.15f;
    c.settledSpeedMmS = 0.5f;
    c.compensateDeadzone = true;
    c.deadzonePwm = 5.55f;
    c.minMovePwm = 6.2f;
    c.maxPwm = M3_PWM_PERCENT; // Techo del PID de M3, en porcentaje.
    c.maxTravelMm = 5000.0f; // Maximo por ORDEN: no es una carrera absoluta referenciada.
    c.travelMarginMm = 2.0f;
    c.moveTimeoutMs = 150000; // E3 hacia END.
    c.noCountsTimeoutMs = 1000; // M3 no hace compresion ni usa closingNoCountsTimeoutMs.
    c.settlingMs = 300; // Confirmar posicion estable con PWM=0.

    // --- H3 y E3: PWM objetivo tras la rampa ---
    c.homePwm = M3_PWM_PERCENT; // H3 hacia HOME.
    c.homeTimeoutMs = 120000; // H3 hacia HOME.
    c.endSeekPwm = M3_PWM_PERCENT; // E3 hacia END.
    return c;
}
