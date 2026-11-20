#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "finales.h"


void finalesBegin() {
    pinMode(FC_HOME, LIMIT_INPUT_MODE);
    pinMode(FC_END, LIMIT_INPUT_MODE);
    if (CONTACT_ENABLED) pinMode(FC_CONTACT, LIMIT_INPUT_MODE);
}
bool homeActivo() { return digitalRead(FC_HOME) == HOME_ACTIVE_LEVEL; }
bool endActivo() { return digitalRead(FC_END) == END_ACTIVE_LEVEL; }
bool contactoActivo() { return CONTACT_ENABLED && digitalRead(FC_CONTACT) == CONTACT_ACTIVE_LEVEL; }
bool algunFinalActivo() { return homeActivo() || endActivo(); }
// Sin impresiones: se puede llamar continuamente sin saturar el monitor.
bool verificarFinales() {
    if (!algunFinalActivo()) return false;
    stopMotor();
    return true;
}
void printFinalesStatus() {
    Serial.printf("# HOME=%d,END=%d,CONTACTO=%d\n", homeActivo(), endActivo(), contactoActivo());
}
