#include <Arduino.h>
#include <esp_arduino_version.h>
#include <math.h>
#include "config.h"
#include "motor.h"

static uint16_t dutyActual = 0;
static int directionActual = -1;
static bool ready = false;
#if ESP_ARDUINO_VERSION_MAJOR < 3
static constexpr uint8_t PWM_CHANNEL = 0;
#endif

static void writeDuty(uint16_t duty) {
    if (!ready) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PWM_PIN, duty);
#else
    ledcWrite(PWM_CHANNEL, duty);
#endif
    dutyActual = duty;
}

bool motorBegin() {
    pinMode(PWM_PIN, OUTPUT);
    digitalWrite(PWM_PIN, LOW);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ready = ledcAttach(PWM_PIN, PWM_FREQ, PWM_BITS);
#else
    ready = ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_BITS) > 0;
    if (ready) ledcAttachPin(PWM_PIN, PWM_CHANNEL);
#endif
    stopMotor();
    return ready;
}
void setPWM(float porcentaje) {
    if (!isfinite(porcentaje)) { stopMotor(); return; }
    porcentaje = constrain(porcentaje, 0.0f, MAX_TEST_PWM);
    writeDuty(static_cast<uint16_t>(lroundf(porcentaje * PWM_MAX / 100.0f)));
}
void setDirection(int dir) {
    // Nunca invertir mientras existe PWM aplicado.
    if (dutyActual != 0) return;
    directionActual = dir > 0 ? 1 : -1;
    digitalWrite(DIR_PIN, directionActual > 0 ? DIR_POSITIVE_LEVEL : (DIR_POSITIVE_LEVEL == LOW ? HIGH : LOW));
}
void stopMotor() { writeDuty(0); }
float getMotorPWM() { return dutyActual * 100.0f / PWM_MAX; }
uint16_t getMotorDuty() { return dutyActual; }
int getMotorDirection() { return directionActual; }
bool motorReady() { return ready; }
