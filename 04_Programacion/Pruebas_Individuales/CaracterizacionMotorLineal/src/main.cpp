#include <Arduino.h>
#include "motor.h"
#include "encoder.h"
#include "finales.h"
#include "caracterizacion.h"
#include "comandos.h"

static char command[96];
static size_t used = 0;
static bool discardLine = false;
static bool startedBusy = false;

static void readCommands() {
    // Trabajo acotado: regresar pronto a revisar finales y muestreo.
    for (unsigned k = 0; k < 32 && Serial.available() > 0; ++k) {
        char c = (char)Serial.read();
        // X sin salto de linea funciona si el monitor envia por caracter.
        // Con send_on_enter se recibe cuando presionas Enter.
        if ((c == 'x' || c == 'X') && used == 0 && !discardLine) {
            cancelTest(); startedBusy = false; continue;
        }
        if (c == '\r' || c == '\n') {
            if (!discardLine && used > 0) {
                command[used] = 0;
                if (startedBusy || caracterizacionBusy()) noteRejectedCommand();
                else processCommand(String(command));
            }
            used = 0; discardLine = false; startedBusy = false;
            continue;
        }
        if (c == '\b' || c == 127) { if (used) --used; continue; }
        if ((unsigned char)c < 32 || discardLine) continue;
        if (used == 0) startedBusy = caracterizacionBusy();
        if (used >= sizeof(command) - 1) { discardLine = true; used = 0; continue; }
        command[used++] = c;
    }
}
void setup() {
    // Establecer salida detenida antes de esperar al USB.
    motorBegin(); encoderBegin(); finalesBegin(); caracterizacionBegin();
    Serial.begin(115200);
    unsigned long started = millis();
    while (!Serial && millis() - started < 2000) delay(10);
    Serial.println("# BANCO_CARACTERIZACION_V3");
    Serial.println("# Datos enviados al terminar. X=parar. ?=ayuda. Q=estado.");
    if (!motorReady()) Serial.println("# ERROR,PWM_NO_INICIALIZADO");
}
void loop() {
    caracterizacionUpdate();
    readCommands();
    caracterizacionUpdate();
    yield();
}
