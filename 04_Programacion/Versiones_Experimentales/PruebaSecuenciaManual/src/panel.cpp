#include "panel.h"
#include "config_panel.h"
#include "telemetria.h"
#include <esp32-hal-rgb-led.h>
#include <stdio.h>

namespace {
struct Input {
    uint8_t pin,level;
    const char* name;
    bool enabled=true;
    bool raw=false,stable=false,pending=true;
    uint32_t changedAt=0;
};
Input inputs[]={
    {PanelConfig::startPin,PanelConfig::startActive,"START"},
    {PanelConfig::stopPin,PanelConfig::stopActive,"STOP",PanelConfig::physicalStopEnabled},
    {PanelConfig::emergencyPin,PanelConfig::emergencyActive,"EMERGENCIA"}
};
bool active=false, iniciado=false;
bool stopConsumido=false, startConsumido=false;
Panel::Indicador indicador=Panel::Indicador::Espera;
uint8_t lastColor=255;
void led() {
    if (!iniciado) return;
    using Panel::Indicador;
    enum : uint8_t { Apagado, Rojo, Verde, Azul, Amarillo };
    const bool emergenciaEstable=inputs[2].stable;
    uint8_t color=emergenciaEstable ? Rojo : active ? Amarillo : Apagado;
    if (PanelConfig::stageIndicatorsEnabled && !emergenciaEstable) {
        const bool pulso=(millis()/400)%2==0;
        color=indicador==Indicador::Bloqueado ? Rojo
            : active ? Amarillo
            : indicador==Indicador::Listo ? Verde
            : !pulso ? Apagado : indicador==Indicador::Movimiento ? Azul : Amarillo;
    }
    if (color==lastColor) return;
    lastColor=color;
    const uint8_t brillo=PanelConfig::brightness;
    neopixelWrite(PanelConfig::rgbPin, color==Rojo || color==Amarillo ? brillo : 0,
        color==Verde || color==Amarillo ? brillo : 0, color==Azul ? brillo : 0);
}
}
void Panel::begin() {
    digitalWrite(PanelConfig::contactorPin,!PanelConfig::contactorActive);
    pinMode(PanelConfig::contactorPin,OUTPUT);
    active=false;
    for (auto& i:inputs) {
        if (!i.enabled) continue;
        pinMode(i.pin,INPUT_PULLUP);
        i.raw=i.stable=digitalRead(i.pin)==i.level;
        i.changedAt=millis();i.pending=true;
    }
    startConsumido=inputs[0].raw;
    stopConsumido=false;
    iniciado=true;
    led();
}
void Panel::on() {
    digitalWrite(PanelConfig::contactorPin,PanelConfig::contactorActive);
    active=true;led();
    printReply("CONTACTOR ON (orden electrica)");
}
void Panel::off() {
    digitalWrite(PanelConfig::contactorPin,!PanelConfig::contactorActive);
    bool was=active;active=false;led();
    if (was) printReply("CONTACTOR OFF");
}
void Panel::update() {
    if (!iniciado) return;
    uint32_t now=millis();
    for (auto& i:inputs) {
        if (!i.enabled) continue;
        bool raw=digitalRead(i.pin)==i.level;
        if (raw!=i.raw) { i.raw=raw;i.changedAt=now; }
        if (i.stable!=raw&&uint32_t(now-i.changedAt)>=PanelConfig::debounceMs) {
            i.stable=raw;i.pending=true;
        }
        if (i.pending) {
            char text[120];
            snprintf(text,sizeof(text),"PANEL %s: %s",
                i.name,i.stable?"ACTIVO":"reposo");
            if (printReply(text)) i.pending=false;
        }
    }
    led();
}
bool Panel::blocked() {
    const auto& stop=inputs[1];
    return emergencyActive() || (stop.enabled && (stop.stable || digitalRead(stop.pin)==stop.level));
}
bool Panel::emergencyActive() {
    const auto& emergency=inputs[2];
    return emergency.stable || digitalRead(emergency.pin)==emergency.level;
}
bool Panel::nuevoStop() {
    const auto& stop=inputs[1];
    if (!stop.enabled) return false;
    if (!stop.raw && !stop.stable && uint32_t(millis()-stop.changedAt)>=PanelConfig::debounceMs)
        stopConsumido=false;
    if (!stop.raw || stopConsumido) return false;
    stopConsumido=true;
    return true;
}
bool Panel::nuevoStart(bool habilitado) {
    const auto& start=inputs[0];
    if (!start.raw && !start.stable && uint32_t(millis()-start.changedAt)>=PanelConfig::debounceMs)
        startConsumido=false;
    if (start.raw && (!habilitado || blocked())) startConsumido=true;
    if (!start.raw || !start.stable || startConsumido) return false;
    startConsumido=true;
    return true;
}
void Panel::indicar(Indicador estado) { indicador=estado;led(); }
bool Panel::report() {
    char text[160];
    const auto& stop=inputs[1];
    const char* stopEstado=stop.enabled ? (stop.stable ? "1" : "0") : "DESHABILITADO";
    snprintf(text,sizeof(text),"PANEL: contactor_orden=%s START=%d STOP=%s EMERGENCIA=%d",
        active?"ON":"OFF",inputs[0].stable,stopEstado,inputs[2].stable);
    return printReply(text);
}
