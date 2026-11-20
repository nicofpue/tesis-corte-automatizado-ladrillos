#include "encoder.h"

void Encoder::begin() {
    pinMode(cfg_.encA, INPUT);
    pinMode(cfg_.encB, INPUT);
    attachInterruptArg(digitalPinToInterrupt(cfg_.encA), interrupt, this, RISING);
}
void IRAM_ATTR Encoder::interrupt(void* argument) {
    auto* encoder = static_cast<Encoder*>(argument);
    const int delta = digitalRead(encoder->cfg_.encB) == LOW ? 1 : -1;
    portENTER_CRITICAL_ISR(&encoder->mux_);
    encoder->count_ += delta;
    ++encoder->risingA_;
    if (delta>0) ++encoder->bLow_; else ++encoder->bHigh_;
    portEXIT_CRITICAL_ISR(&encoder->mux_);
}
int32_t Encoder::count() const {
    portENTER_CRITICAL(&mux_);
    const int32_t result = count_;
    portEXIT_CRITICAL(&mux_);
    return result;
}
void Encoder::zero() {
    portENTER_CRITICAL(&mux_);
    count_ = 0;
    portEXIT_CRITICAL(&mux_);
}
float Encoder::position() const {
    return cfg_.encoderSign * static_cast<float>(count()) * cfg_.mmPerCount();
}
EncoderDiagnostic Encoder::diagnostic() const {
    EncoderDiagnostic d;
    portENTER_CRITICAL(&mux_);
    d.count=count_;d.risingA=risingA_;d.bLow=bLow_;d.bHigh=bHigh_;
    portEXIT_CRITICAL(&mux_);
    d.a=digitalRead(cfg_.encA);d.b=digitalRead(cfg_.encB);
    return d;
}
