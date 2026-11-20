// =====================================================================
// encoder.cpp - Conteo de pulsos del encoder
// ---------------------------------------------------------------------
// =====================================================================
#include "encoder.h"

// Configura A y B como entradas y asocia la interrupcion al flanco de subida de A.
void Encoder::begin() {
    pinMode(cfg_.encA, INPUT);
    pinMode(cfg_.encB, INPUT);
    attachInterruptArg(digitalPinToInterrupt(cfg_.encA), interrupt, this, RISING);
}
// Interrupcion (en IRAM para responder rapido): B en bajo = +1 cuenta, B en alto = -1.
// Tambien cuenta flancos y niveles de B para el diagnostico (comando E).
void IRAM_ATTR Encoder::interrupt(void* argument) {
    auto* encoder = static_cast<Encoder*>(argument);
    const int delta = digitalRead(encoder->cfg_.encB) == LOW ? 1 : -1;
    portENTER_CRITICAL_ISR(&encoder->mux_);
    encoder->count_ += delta;
    ++encoder->risingA_;
    if (delta>0) ++encoder->bLow_; else ++encoder->bHigh_;
    portEXIT_CRITICAL_ISR(&encoder->mux_);
}
// Lectura protegida del contador.
int32_t Encoder::count() const {
    portENTER_CRITICAL(&mux_);
    const int32_t result = count_;
    portEXIT_CRITICAL(&mux_);
    return result;
}
// Pone el contador en cero (solo al confirmar HOME).
void Encoder::zero() {
    portENTER_CRITICAL(&mux_);
    count_ = 0;
    portEXIT_CRITICAL(&mux_);
}
// Posicion en mm: cuentas x signo del eje x escala (mm por cuenta).
float Encoder::position() const {
    return cfg_.encoderSign * static_cast<float>(count()) * cfg_.mmPerCount();
}
// Copia de contadores y niveles actuales de A y B para el comando E.
EncoderDiagnostic Encoder::diagnostic() const {
    EncoderDiagnostic d;
    portENTER_CRITICAL(&mux_);
    d.count=count_;d.risingA=risingA_;d.bLow=bLow_;d.bHigh=bHigh_;
    portEXIT_CRITICAL(&mux_);
    d.a=digitalRead(cfg_.encA);d.b=digitalRead(cfg_.encB);
    return d;
}
