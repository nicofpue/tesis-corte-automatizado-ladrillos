#pragma once
#include "config_eje.h"

// Contadores crudos desde arranque: HOME pone cero a count, no a las estadisticas.
struct EncoderDiagnostic {
    int32_t count=0;
    uint32_t risingA=0, bLow=0, bHigh=0;
    uint8_t a=0,b=0;
};

class Encoder {
public:
    explicit Encoder(const AxisConfig& cfg) : cfg_(cfg) {}
    Encoder(const Encoder&) = delete;
    Encoder& operator=(const Encoder&) = delete;
    void begin();
    int32_t count() const;
    void zero();
    float position() const;
    EncoderDiagnostic diagnostic() const;
private:
    static void IRAM_ATTR interrupt(void* argument);
    AxisConfig cfg_;
    volatile int32_t count_ = 0;
    volatile uint32_t risingA_=0, bLow_=0, bHigh_=0;
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
