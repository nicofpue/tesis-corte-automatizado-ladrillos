#pragma once
#include <Arduino.h>

// Seleccionar EXACTAMENTE un bloque. Pines y magnitudes son propuestas sin validar.
#define MOTOR_M1
// #define MOTOR_M2
// #define MOTOR_M3
#if (defined(MOTOR_M1) + defined(MOTOR_M2) + defined(MOTOR_M3)) != 1
#error "Seleccione exactamente un motor M1/M2/M3"
#endif
constexpr const char* FIRMWARE_VERSION = "3.0.0";

#if defined(MOTOR_M1)
// M1: comprobar fisicamente TODOS estos valores antes de habilitar ensayos.
constexpr const char* MOTOR_NAME = "M1";
constexpr uint8_t PWM_PIN = 40, DIR_PIN = 39, ENC_A = 9, ENC_B = 10;
constexpr uint8_t FC_HOME = 4, FC_END = 5, FC_CONTACT = 6;
constexpr bool CONTACT_ENABLED = true;
constexpr uint8_t HOME_ACTIVE_LEVEL = LOW, END_ACTIVE_LEVEL = LOW, CONTACT_ACTIVE_LEVEL = LOW;
constexpr uint8_t LIMIT_INPUT_MODE = INPUT_PULLUP; // comprobar resistencias externas / INPUT_PULLUP
constexpr uint8_t DIR_POSITIVE_LEVEL = LOW;
constexpr int CLAMP_DIR = 1, OPEN_DIR = -1, ENCODER_SIGN = 1;
constexpr float PPR = 1000.0f, LEAD_MM_REV = 8.0f, GEAR_RATIO = 1.0f;
constexpr float MAX_TEST_PWM = 30.0f; // CONFIGURAR limite propio del eje, porcentaje
constexpr float MAX_TRAVEL_MM = 150.0f; // CONFIGURAR recorrido maximo por ensayo, mm
constexpr bool AXIS_CONFIG_VERIFIED = true; // true solo despues de verificar fisicamente

#elif defined(MOTOR_M2)
// M2: comprobar fisicamente TODOS estos valores antes de habilitar ensayos.
constexpr const char* MOTOR_NAME = "M2";
constexpr uint8_t PWM_PIN = 42, DIR_PIN = 41, ENC_A = 11, ENC_B = 12;
constexpr uint8_t FC_HOME = 17, FC_END = 18, FC_CONTACT = 8;
constexpr bool CONTACT_ENABLED = true;
constexpr uint8_t HOME_ACTIVE_LEVEL = LOW, END_ACTIVE_LEVEL = LOW, CONTACT_ACTIVE_LEVEL = LOW;
constexpr uint8_t LIMIT_INPUT_MODE = INPUT_PULLUP; // comprobar resistencias externas / INPUT_PULLUP
constexpr uint8_t DIR_POSITIVE_LEVEL = LOW;
constexpr int CLAMP_DIR = 1, OPEN_DIR = -1, ENCODER_SIGN = 1;
constexpr float PPR = 1000.0f, LEAD_MM_REV = 8.0f, GEAR_RATIO = 1.0f;
constexpr float MAX_TEST_PWM = 30.0f; // CONFIGURAR limite propio del eje, porcentaje
constexpr float MAX_TRAVEL_MM = 150.0f; // CONFIGURAR recorrido maximo por ensayo, mm
constexpr bool AXIS_CONFIG_VERIFIED = true; // true solo despues de verificar fisicamente

#elif defined(MOTOR_M3)
// M3: comprobar fisicamente TODOS estos valores antes de habilitar ensayos.
constexpr const char* MOTOR_NAME = "M3";
constexpr uint8_t PWM_PIN = 1, DIR_PIN = 2, ENC_A = 13, ENC_B = 14;
constexpr uint8_t FC_HOME = 7, FC_END = 15, FC_CONTACT = 16;
constexpr bool CONTACT_ENABLED = false;
constexpr uint8_t HOME_ACTIVE_LEVEL = LOW, END_ACTIVE_LEVEL = LOW, CONTACT_ACTIVE_LEVEL = LOW;
constexpr uint8_t LIMIT_INPUT_MODE = INPUT_PULLUP; // comprobar resistencias externas / INPUT_PULLUP
constexpr uint8_t DIR_POSITIVE_LEVEL = LOW;
constexpr int CLAMP_DIR = 1, OPEN_DIR = -1, ENCODER_SIGN = 1;
constexpr float PPR = 1000.0f, LEAD_MM_REV = 8.0f, GEAR_RATIO = 10.0f;
constexpr float MAX_TEST_PWM = 80.0f; // CONFIGURAR limite propio del eje, porcentaje
constexpr float MAX_TRAVEL_MM = 5000.0f; // CONFIGURAR recorrido maximo por ensayo, mm
constexpr bool AXIS_CONFIG_VERIFIED = true; // true solo despues de verificar fisicamente
#endif
static_assert(CLAMP_DIR == -OPEN_DIR && (CLAMP_DIR == 1 || CLAMP_DIR == -1), "Sentidos incompatibles");
static_assert(ENCODER_SIGN == 1 || ENCODER_SIGN == -1, "Signo encoder invalido");
static_assert(MAX_TEST_PWM >= 0 && MAX_TEST_PWM <= 100 && MAX_TRAVEL_MM >= 0, "Limites invalidos");
constexpr bool CONTACT_COMMANDS_ALLOWED = MOTOR_NAME[1] != '3';
constexpr uint32_t CONTACT_RELEASE_CONFIRM_US = 20000; // validar rebote fisicamente
constexpr size_t INPUT_EVENT_CAPACITY = 512;
constexpr uint32_t PWM_FREQ = 20000;
constexpr uint8_t PWM_BITS = 10;
constexpr uint16_t PWM_MAX = (1U << PWM_BITS) - 1;
// Encoder x1: RISING de A; B determina sentido. PPR debe corresponder a x1.
constexpr float COUNTS_PER_OUTPUT_REV = PPR * GEAR_RATIO;
constexpr float MM_PER_COUNT = LEAD_MM_REV / COUNTS_PER_OUTPUT_REV;
static_assert(PPR > 0 && GEAR_RATIO > 0 && LEAD_MM_REV > 0, "Escala invalida");
constexpr uint32_t SAMPLE_US = 5000;
constexpr uint32_t PRE_MS = 300, REST_CHECK_MS = 100, COAST_MS = 700;
constexpr uint32_t MAX_STEP_TIME_MS = 15000, MAX_TEST_TIME_MS = 15000;
constexpr uint32_t MAX_RUN_MS = MAX_STEP_TIME_MS > MAX_TEST_TIME_MS ? MAX_STEP_TIME_MS : MAX_TEST_TIME_MS;
constexpr size_t SAMPLE_CAPACITY = ((PRE_MS + MAX_RUN_MS + COAST_MS) * 1000UL) / SAMPLE_US + 64;
static_assert(SAMPLE_US >= 1000, "Revisar memoria al reducir el muestreo");
