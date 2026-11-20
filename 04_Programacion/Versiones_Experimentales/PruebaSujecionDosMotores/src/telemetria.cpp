#include "telemetria.h"
#include <stdio.h>
#include <string.h>
namespace {
constexpr unsigned CAPACITY=16, LINE_SIZE=192;
char queue[CAPACITY][LINE_SIZE];
unsigned head=0,tail=0,size=0,offset=0;
}
bool printReply(const char* text) {
    if (size==CAPACITY) return false;
    snprintf(queue[tail],LINE_SIZE,"%s\n",text);
    tail=(tail+1)%CAPACITY; ++size; return true;
}
bool printStatus(const Axis& a) {
    char text[LINE_SIZE];
    snprintf(text,sizeof(text),"%s ref=%d x=%.3f r=%.3f e=%.3f u=%.2f H=%d E=%d C=%d cuentas=%ld ayuda_sync=%.2f",
        a.stateName(),a.isHomed(),a.position(),a.target(),a.target()-a.position(),a.pwm(),
        a.homePressed(),a.endPressed(),a.contactPressed(),static_cast<long>(a.counts()),a.syncBoost());
    return printReply(text);
}
bool printEncoder(const Axis& a) {
    char text[LINE_SIZE];
    snprintf(text,sizeof(text),"Encoder=%ld x=%.3f mm",static_cast<long>(a.counts()),a.position());
    return printReply(text);
}
bool printEncoderDiagnostic(const EncoderDiagnostic& d,const AxisConfig& c) {
    char text[LINE_SIZE];
    snprintf(text,sizeof(text),"ENC A=GPIO%u nivel=%u B=GPIO%u nivel=%u cuentas=%ld flancosA=%lu B_LOW=%lu B_HIGH=%lu",
        c.encA,d.a,c.encB,d.b,static_cast<long>(d.count),static_cast<unsigned long>(d.risingA),
        static_cast<unsigned long>(d.bLow),static_cast<unsigned long>(d.bHigh));
    return printReply(text);
}
bool printPins(const Axis& a) {
    const auto& c=a.config();char text[LINE_SIZE];
    snprintf(text,sizeof(text),"PINES PWM=%u DIR=%u HOME=%u END=%u CONTACTO=%u ENC_A=%u ENC_B=%u; PWM_min=%.1f max_PID=%.1f max_CT=%.1f cierre_D=%.1f; sin_avance cierre=%lu apertura=%lu ms",
        c.pwmPin,c.dirPin,c.homePin,c.endPin,c.contactPin,c.encA,c.encB,c.minMovePwm,c.maxPwm,c.heldMaxPwm,c.contactPwm,
        static_cast<unsigned long>(c.closingNoCountsTimeoutMs),static_cast<unsigned long>(c.noCountsTimeoutMs));
    return printReply(text);
}
void serviceTelemetry() {
    if (!size||!Serial) return;
    int room=Serial.availableForWrite(); if (room<=0) return;
    size_t remaining=strlen(queue[head])-offset;
    size_t chunk=remaining<static_cast<size_t>(room)?remaining:static_cast<size_t>(room);
    if (chunk>64) chunk=64;
    offset+=Serial.write(reinterpret_cast<const uint8_t*>(queue[head]+offset),chunk);
    if (offset==strlen(queue[head])) { offset=0;head=(head+1)%CAPACITY;--size; }
}
