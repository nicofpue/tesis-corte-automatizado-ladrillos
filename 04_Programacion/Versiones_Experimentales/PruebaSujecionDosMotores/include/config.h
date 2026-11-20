#pragma once
// ARCHIVO LEGADO: el programa actual NO incluye este archivo.
// Ajustes activos: config_eje.h (motores) y config_geometria.h (caras/centro).
// Se conserva su contenido anterior; sus constantes no gobiernan esta revision.
#include <Arduino.h>
// Distancias mm, tiempos ms y PWM %. Todos los ajustes estan aqui.
struct Config {
    uint8_t pwm,dir,a,b,home,end,contact,channel;
    float travel;
    uint8_t active=LOW, inputMode=INPUT_PULLUP, closeDir=LOW;
    int8_t encoderSign=-1;
    float mmPerCount=0.008f, homePwm=15, maxPwm=20;
    // Valores iniciales del proyecto base, no sintonizados: PID con I y D desactivadas.
    float kp=0.60f, ki=0, kd=0, integralLimit=20, derivativeFilterS=0.03f;
    float deadzonePwm=5.55f, minMovePwm=6.2f;
};
inline const Config CONFIG[]={{40,39,9,10,4,5,6,0,68},{42,41,11,12,17,18,8,1,74}};
constexpr float D0_MM=252; // Entre caras en HOME, SIN compresion.
constexpr float TRAVEL_MARGIN_MM=2; // Solo corte de emergencia; no amplia objetivos.
constexpr float TOLERANCE_MM=0.15f, SETTLED_SPEED_MM_S=0.5f;
constexpr uint32_t PWM_HZ=20000;
constexpr uint8_t PWM_BITS=10;
constexpr uint32_t HOME_TIMEOUT_MS=15000, MOVE_TIMEOUT_MS=30000;
constexpr uint32_t SENSOR_CONFIRM_MS=30, HOME_CONFIRM_TIMEOUT_MS=2000, SETTLING_MS=300;
constexpr uint32_t CONTROL_US=10000, MAX_LOOP_GAP_US=100000, DIR_PAUSE_US=20000;
// Proteccion original para posicionamiento SIN compresion; editable, no se usa en HOME.
constexpr uint32_t NO_COUNTS_MS=1000;
constexpr uint32_t SERIAL_BAUD=115200;
