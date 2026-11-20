#include "panel.h"
#include "config_panel.h"
#include "telemetria.h"
#include <esp32-hal-rgb-led.h>
#include <stdio.h>

namespace {
struct Input {
    uint8_t pin,level;
    const char* name;
    bool raw=false,stable=false,pending=true;
    uint32_t changedAt=0;
};
Input inputs[]={
    {PanelConfig::startPin,PanelConfig::startActive,"START"},
    {PanelConfig::stopPin,PanelConfig::stopActive,"STOP"},
    {PanelConfig::emergencyPin,PanelConfig::emergencyActive,"EMERGENCIA"}
};
bool active=false;
uint8_t lastColor=255;
void led() {
    // Rojo prioritario; amarillo indica orden electrica, no realimentacion del contactor.
    uint8_t color=inputs[2].stable?2:active?1:0;
    if (color==lastColor) return;
    lastColor=color;
    neopixelWrite(PanelConfig::rgbPin,color?PanelConfig::brightness:0,
        color==1?PanelConfig::brightness:0,0);
}
}
void Panel::begin() {
    digitalWrite(PanelConfig::contactorPin,!PanelConfig::contactorActive);
    pinMode(PanelConfig::contactorPin,OUTPUT);
    active=false;
    for (auto& i:inputs) {
        pinMode(i.pin,INPUT_PULLUP);
        i.raw=i.stable=digitalRead(i.pin)==i.level;
        i.changedAt=millis();i.pending=true;
    }
    led();
}
void Panel::on() {
    digitalWrite(PanelConfig::contactorPin,PanelConfig::contactorActive);
    active=true;led();
    printReply("CONTACTOR ON; sin temporizador");
}
void Panel::off() {
    digitalWrite(PanelConfig::contactorPin,!PanelConfig::contactorActive);
    bool was=active;active=false;led();
    if (was) printReply("CONTACTOR OFF");
}
void Panel::update() {
    uint32_t now=millis();
    for (auto& i:inputs) {
        bool raw=digitalRead(i.pin)==i.level;
        if (raw!=i.raw) { i.raw=raw;i.changedAt=now; }
        if (i.stable!=raw&&uint32_t(now-i.changedAt)>=PanelConfig::debounceMs) {
            i.stable=raw;i.pending=true;
        }
        if (i.pending) {
            char text[120];
            snprintf(text,sizeof(text),"PANEL %s: %s (solo aviso; sin accion sobre motores/contactor)",
                i.name,i.stable?"ACTIVO":"reposo");
            if (printReply(text)) i.pending=false;
        }
    }
    led();
}
bool Panel::report() {
    char text[160];
    snprintf(text,sizeof(text),"PANEL: contactor_orden=%s START=%d STOP=%d EMERGENCIA=%d; entradas solo aviso",
        active?"ON":"OFF",inputs[0].stable,inputs[1].stable,inputs[2].stable);
    return printReply(text);
}
