#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "finales.h"
#include "caracterizacion.h"

// Adquisicion en RAM. No se imprime mientras el carro se mueve.
struct Sample {
    uint32_t t, dt;
    int32_t count, dc;
    uint16_t duty;
};
static Sample samples[SAMPLE_CAPACITY];
struct InputEvent { uint32_t t; uint8_t input; bool active; };
static InputEvent inputs[INPUT_EVENT_CAPACITY];
static size_t inputCount = 0, inputOut = 0;
static bool inputOverflow = false, previousInputs[3] = {};
static bool releasePending = false;
static uint32_t releaseSince = 0;

enum class State { IDLE, PRE, ON, COAST, DUMP };
static State state = State::IDLE;
static uint32_t trial = 0, t0 = 0, lastUs = 0, onUs = 0, offUs = 0;
static uint32_t phaseUs = 0, lastMotionUs = 0, durationMs = 0;
static uint32_t rejected = 0, maxDt = 0;
static int32_t startCount = 0, lastCount = 0, observedCount = 0;
static long target = 0;
static int direction = 1;
static float requested = 0;
static char kind = 'S';
static const char* reason = "";
static bool switchedOn = false, overflow = false, quietAtEnd = false;
static size_t n = 0, outIndex = 0;
static unsigned outStage = 0;
static char line[512];
static size_t lineLen = 0, lineSent = 0;

static void recordInputs(uint32_t now, bool initial = false) {
    bool values[] = {homeActivo(), endActivo(), contactoActivo()};
    for (uint8_t i = 0; i < 3; ++i) {
        if (i == 2 && !CONTACT_ENABLED) continue;
        if (initial || values[i] != previousInputs[i]) {
            if (inputCount < INPUT_EVENT_CAPACITY) inputs[inputCount++] = {now - t0, i, values[i]};
            else inputOverflow = true;
            previousInputs[i] = values[i];
        }
    }
}

bool caracterizacionBusy() { return state != State::IDLE; }
void noteRejectedCommand() { ++rejected; }
void caracterizacionBegin() { state = State::IDLE; stopMotor(); }

static void capture(uint32_t now, bool force = false) {
    uint32_t dt = now - lastUs;
    if (!force && dt < SAMPLE_US) return;
    if (n && dt == 0) return;
    if (n >= SAMPLE_CAPACITY) { overflow = true; return; }
    int32_t count = getEncoder();
    samples[n++] = {now - t0, dt, count, count - lastCount, getMotorDuty()};
    if (dt > maxDt) maxDt = dt;
    lastCount = count;
    lastUs = now;
}

static void finishDrive(const char* why) {
    stopMotor(); // La parada precede al registro y a cualquier impresion.
    uint32_t now = micros();
    offUs = now - t0;
    reason = why;
    phaseUs = now;
    state = State::COAST;
    capture(now, true);
}

void cancelTest() {
    stopMotor();
    if (state == State::PRE || state == State::ON) finishDrive("STOP_USUARIO");
    // En COAST conserva el motivo original; en DUMP conserva los datos.
}

static void startTest(char type, float pwm, uint32_t ms, long counts, int dir) {
    if (caracterizacionBusy()) { noteRejectedCommand(); return; }
    if (!AXIS_CONFIG_VERIFIED || MAX_TEST_PWM <= 0 || MAX_TRAVEL_MM <= 0) {
        Serial.println("# ERROR,CONFIGURAR_Y_VERIFICAR_EJE"); return;
    }
    if ((type == 'A' || type == 'R') && (!CONTACT_COMMANDS_ALLOWED || !CONTACT_ENABLED)) {
        Serial.println("# ERROR,CONTACTO_NO_DISPONIBLE"); return;
    }
    if ((type == 'A' && dir != CLAMP_DIR) || (type == 'R' && dir != OPEN_DIR)) {
        Serial.println("# ERROR,DIRECCION_NO_PERMITIDA"); return;
    }
    if ((type == 'R') != contactoActivo()) {
        Serial.println(type == 'R' ? "# ERROR,CONTACTO_INACTIVO" : "# ERROR,CONTACTO_ACTIVO"); return;
    }
    if (!motorReady()) { Serial.println("# ERROR,PWM_NO_INICIALIZADO"); return; }
    if (!isfinite(pwm) || pwm < 0 || pwm > MAX_TEST_PWM ||
        (dir != 1 && dir != -1) || ms == 0 || ms > MAX_RUN_MS ||
        ((type == 'A' || type == 'R') && pwm <= 0) ||
        (type == 'C' && (counts <= 0 || pwm == 0 || counts * MM_PER_COUNT >= MAX_TRAVEL_MM))) {
        Serial.println("# ERROR,PARAMETROS_O_LIMITE_RECORRIDO"); return;
    }
    if (verificarFinales()) { Serial.println("# ERROR,FINAL_ACTIVO"); return; }
    stopMotor();
    setDirection(dir);
    ++trial;
    kind = type; requested = pwm; durationMs = ms; target = counts; direction = dir;
    n = 0; rejected = 0; maxDt = 0; overflow = false; switchedOn = false;
    quietAtEnd = false; reason = "";
    onUs = 0; offUs = 0;
    startCount = lastCount = observedCount = getEncoder();
    t0 = lastUs = phaseUs = lastMotionUs = micros();
    inputCount = inputOut = 0; inputOverflow = false; releasePending = false;
    recordInputs(t0, true);
    state = State::PRE;
    capture(t0, true);
}
void runStep(float pwm, unsigned long ms, int dir) {
    if (ms == 0 || ms > MAX_STEP_TIME_MS) {
        Serial.println("# ERROR,TIEMPO_FUERA_DE_RANGO"); return;
    }
    startTest('S', pwm, ms, 0, dir);
}
void runCalibration(float pwm, long counts, int dir) {
    startTest('C', pwm, MAX_TEST_TIME_MS, counts, dir);
}

void runApproach(float pwm, unsigned long ms, int dir) { startTest('A', pwm, ms, 0, dir); }
void runRelease(float pwm, unsigned long ms) { startTest('R', pwm, ms, 0, OPEN_DIR); }

// Cada llamada transmite solo lo que cabe ahora. No espera al puerto.
static void dumpUpdate() {
    if (lineSent < lineLen) {
        int room = Serial.availableForWrite();
        if (room <= 0) return;
        size_t bytes = lineLen - lineSent;
        if (bytes > static_cast<size_t>(room)) bytes = room;
        if (bytes > 64) bytes = 64;
        lineSent += Serial.write(reinterpret_cast<const uint8_t*>(line) + lineSent, bytes);
        return;
    }
    int len = 0;
    switch (outStage) {
    case 0:
        len = snprintf(line, sizeof(line), "\n# START,intento=%lu,tipo=%c,pwm_pedido=%.4f,dir=%d\n",
            (unsigned long)trial, kind, requested, direction); ++outStage; break;
    case 1:
        len = snprintf(line, sizeof(line), "# CONFIG,motor=%s,version=%s,cpr_motor=%.3f,reductor=%.3f,cpr_salida=%.3f,lead=%.5f,mm_count=%.9f,sample_us=%lu,max_travel_mm=%.3f\n",
            MOTOR_NAME, FIRMWARE_VERSION, PPR, GEAR_RATIO, COUNTS_PER_OUTPUT_REV, LEAD_MM_REV, MM_PER_COUNT, (unsigned long)SAMPLE_US, MAX_TRAVEL_MM); ++outStage; break;
    case 2:
        len = snprintf(line, sizeof(line), "# EVENT,on_us=%lu,off_us=%lu,aplicado=%d,max_dt_us=%lu\n",
            (unsigned long)onUs, (unsigned long)offUs, switchedOn,
            (unsigned long)maxDt); ++outStage; break;
    case 3:
        len = snprintf(line, sizeof(line), "intento,t_us,pwm,dir,counts,delta_counts,x_mm,v_mm_s,rpm_salida,vueltas_salida\n");
        ++outStage; break;
    case 4:
        if (outIndex < n) {
            const Sample &s = samples[outIndex++];
            float v = s.dt ? s.dc * MM_PER_COUNT * 1000000.0f / s.dt : 0;
            float rpm = s.dt ? s.dc * 60000000.0f / (COUNTS_PER_OUTPUT_REV * s.dt) : 0;
            len = snprintf(line, sizeof(line), "%lu,%lu,%.4f,%d,%ld,%ld,%.5f,%.5f,%.3f,%.6f\n",
                (unsigned long)trial, (unsigned long)s.t, s.duty * 100.0f / PWM_MAX,
                direction, (long)s.count, (long)(s.count - startCount),
                (s.count - startCount) * MM_PER_COUNT, v, rpm,
                (s.count - startCount) / COUNTS_PER_OUTPUT_REV);
        } else ++outStage;
        break;
    case 5:
        if (inputOut < inputCount) {
            const InputEvent &e = inputs[inputOut++];
            const char* names[] = {"HOME", "END", "CONTACTO"};
            len = snprintf(line, sizeof(line), "# INPUT,intento=%lu,t_us=%lu,input=%s,activo=%d\n",
                (unsigned long)trial, (unsigned long)e.t, names[e.input], e.active);
        } else ++outStage;
        break;
    case 6:
        len = snprintf(line, sizeof(line), "# END,intento=%lu,motivo=%s,quiet=%d,overflow=%d,input_overflow=%d,rechazados=%lu,final_counts=%ld\n",
            (unsigned long)trial, reason, quietAtEnd, overflow || inputOverflow, inputOverflow,
            (unsigned long)rejected, (long)(lastCount - startCount)); ++outStage; break;
    case 7:
        len = snprintf(line, sizeof(line), "# LISTO\n"); ++outStage; break;
    default:
        state = State::IDLE; return;
    }
    lineLen = len > 0 ? static_cast<size_t>(len) : 0;
    if (lineLen >= sizeof(line)) lineLen = sizeof(line) - 1;
    lineSent = 0;
}

void caracterizacionUpdate() {
    uint32_t now = micros();
    if (state == State::PRE || state == State::ON || state == State::COAST) recordInputs(now);
    bool home = homeActivo(), end = endActivo();
    if (home || end) {
        stopMotor();
        if (state == State::PRE || state == State::ON)
            finishDrive(home && end ? "AMBOS_FC" : (home ? "HOME" : "END"));
        else if (state == State::COAST &&
                 (strcmp(reason, "TIEMPO_COMPLETADO") == 0 || strcmp(reason, "CUENTAS_COMPLETADAS") == 0 || strcmp(reason, "CONTACTO_LIBERADO") == 0 || strcmp(reason, "CONTACTO") == 0))
            reason = home && end ? "AMBOS_FC" : (home ? "HOME" : "END");
    }
    if (state == State::IDLE) return;
    if (state == State::DUMP) { dumpUpdate(); return; }
    if (inputOverflow && (state == State::PRE || state == State::ON)) finishDrive("BUFFER_INPUT_LLENO");
    if (kind != 'R' && contactoActivo()) {
        if (state == State::PRE || state == State::ON) finishDrive("CONTACTO");
        else if (state == State::COAST &&
            (strcmp(reason, "TIEMPO_COMPLETADO") == 0 || strcmp(reason, "CUENTAS_COMPLETADAS") == 0 || strcmp(reason, "CONTACTO_LIBERADO") == 0 || strcmp(reason, "CONTACTO") == 0)) reason = "CONTACTO";
    }
    now = micros();
    int32_t count = getEncoder();
    if (count != observedCount) { lastMotionUs = now; observedCount = count; }

    if (state == State::ON) {
        if (fabsf((count - startCount) * MM_PER_COUNT) >= MAX_TRAVEL_MM)
            finishDrive("LIMITE_RECORRIDO");
        else if (kind == 'C' && labs((long)(count - startCount)) >= target)
            finishDrive("CUENTAS_COMPLETADAS");
        else if (now - phaseUs >= durationMs * 1000UL)
            finishDrive(kind == 'S' ? "TIEMPO_COMPLETADO" : "TIMEOUT");
    }
    // R: apertura exclusivamente; confirmar liberacion continua, sin homing ni rearme.
    if (kind == 'R' && state == State::ON) {
        if (contactoActivo()) releasePending = false;
        else if (!releasePending) { releasePending = true; releaseSince = now; }
        else if (now - releaseSince >= CONTACT_RELEASE_CONFIRM_US) finishDrive("CONTACTO_LIBERADO");
    }
    if (state == State::COAST && fabsf((count - startCount) * MM_PER_COUNT) >= MAX_TRAVEL_MM &&
        (strcmp(reason, "TIEMPO_COMPLETADO") == 0 || strcmp(reason, "CUENTAS_COMPLETADAS") == 0 || strcmp(reason, "CONTACTO_LIBERADO") == 0 || strcmp(reason, "CONTACTO") == 0))
        reason = "LIMITE_RECORRIDO";
    now = micros();
    capture(now);
    if (overflow && (state == State::PRE || state == State::ON)) finishDrive("BUFFER_LLENO");

    if (state == State::PRE && now - phaseUs >= PRE_MS * 1000UL) {
        if (now - lastMotionUs < REST_CHECK_MS * 1000UL) { finishDrive("NO_REPOSO_INICIAL"); return; }
        if (verificarFinales()) { finishDrive("FINAL_PREARRANQUE"); return; }
        if (kind == 'R' && !contactoActivo()) { finishDrive("CONTACTO_INACTIVO_PREARRANQUE"); return; }
        if (kind != 'R' && contactoActivo()) { finishDrive("CONTACTO"); return; }
        setPWM(requested);
        phaseUs = micros(); onUs = phaseUs - t0; switchedOn = true;
        state = State::ON;
        capture(phaseUs, true);
    } else if (state == State::COAST && now - phaseUs >= COAST_MS * 1000UL) {
        stopMotor();
        capture(micros(), true);
        quietAtEnd = now - lastMotionUs >= REST_CHECK_MS * 1000UL;
        outStage = 0; outIndex = 0; lineLen = lineSent = 0;
        state = State::DUMP;
    }
}

