#include <Arduino.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "finales.h"
#include "caracterizacion.h"
#include "comandos.h"

static bool parseFloatStrict(const char* s, float &v) {
    errno = 0; char* end;
    v = strtof(s, &end);
    while (isspace((unsigned char)*end)) ++end;
    return end != s && *end == 0 && errno != ERANGE && isfinite(v);
}
static bool parseLongStrict(const char* s, long &v) {
    errno = 0; char* end;
    v = strtol(s, &end, 10);
    while (isspace((unsigned char)*end)) ++end;
    return end != s && *end == 0 && errno != ERANGE;
}
void processCommand(String command) {
    command.trim(); command.toUpperCase();
    if (command.length() == 0) return;
    if (command == "X") {
        cancelTest();
        if (!caracterizacionBusy()) Serial.println("# MOTOR_DETENIDO");
        return;
    }
    if (caracterizacionBusy()) { noteRejectedCommand(); return; }
    if (command == "Q") { printEncoderStatus(); printFinalesStatus(); return; }
    if (command == "Z") {
        stopMotor(); zeroEncoder(); Serial.println("# CERO_LOCAL; NO ES HOMING"); return;
    }
    if (command == "?") {
        Serial.println("# Z | Q | X | S,PWM,TIEMPO_MS,DIR | C,PWM,CUENTAS,DIR | A,PWM,TIEMPO_MAX_MS,DIR | R,PWM,TIEMPO_MAX_MS");
        Serial.printf("# PWM_MAX=%.2f,RECORRIDO_MAX_MM=%.2f,TIEMPO_MAX_MS=%lu\n",
            MAX_TEST_PWM, MAX_TRAVEL_MM, (unsigned long)MAX_STEP_TIME_MS);
        return;
    }
    char text[96]; command.toCharArray(text, sizeof(text));
    char* fields[4] = {text, nullptr, nullptr, nullptr};
    unsigned count = 1;
    for (char* p = text; *p; ++p) {
        if (*p == ',') {
            if (count >= 4) { Serial.println("# ERROR,FORMATO"); return; }
            *p = 0; fields[count++] = p + 1;
        }
    }
    float pwm; long value, dir = OPEN_DIR;
    bool release = count == 3 && strcmp(fields[0], "R") == 0;
    if ((!release && count != 4) || strlen(fields[0]) != 1 ||
        (!release && fields[0][0] != 'S' && fields[0][0] != 'C' && fields[0][0] != 'A') ||
        !parseFloatStrict(fields[1], pwm) || !parseLongStrict(fields[2], value) ||
        (!release && !parseLongStrict(fields[3], dir)) || value <= 0 ||
        (dir != 1 && dir != -1) || pwm < 0 || pwm > MAX_TEST_PWM) {
        Serial.println("# ERROR,COMANDO_INVALIDO; use ?"); return;
    }
    if (release) runRelease(pwm, (unsigned long)value);
    else if (fields[0][0] == 'A') runApproach(pwm, (unsigned long)value, (int)dir);
    else if (fields[0][0] == 'S') runStep(pwm, (unsigned long)value, (int)dir);
    else runCalibration(pwm, value, (int)dir);
}
