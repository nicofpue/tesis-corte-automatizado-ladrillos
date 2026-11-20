// =====================================================================
// panel.cpp - Entradas del panel, contactor y LED
// ---------------------------------------------------------------------
// =====================================================================
#include "panel.h"
#include "config_panel.h"
#include "telemetria.h"
#include <esp32-hal-rgb-led.h>
#include <stdio.h>

namespace {
// Estado de cada entrada: lectura cruda, valor estable (antirrebote) y si falta informar el cambio.
struct Input {
    uint8_t pin,level;
    const char* name;
    bool enabled=true;
    bool raw=false,stable=false,pending=true;
    uint32_t changedAt=0;
};
// Orden fijo: [0] START, [1] STOP, [2] EMERGENCIA.
Input inputs[]={
    {PanelConfig::startPin,PanelConfig::startActive,"START"},
    {PanelConfig::stopPin,PanelConfig::stopActive,"STOP",PanelConfig::physicalStopEnabled},
    {PanelConfig::emergencyPin,PanelConfig::emergencyActive,"EMERGENCIA"}
};
bool active=false, iniciado=false;
bool stopConsumido=false, startConsumido=false;
Panel::Color colorPedido=Panel::Color::Amarillo;
bool titilaPedido=true;
Panel::Color lastColor=Panel::Color::Apagado;
bool ledEscrito=false;
// Escribe el color pedido en el LED. EMERGENCIA activa fuerza rojo fijo.
// Solo escribe cuando el color cambia.
void led() {
    if (!iniciado) return;
    using Panel::Color;
    Color color=colorPedido;
    if (inputs[2].stable) color=Color::Rojo;          // EMERGENCIA: rojo fijo.
    else if (titilaPedido && (millis()/PanelConfig::blinkHalfPeriodMs)%2) color=Color::Apagado;
    if (ledEscrito && color==lastColor) return;
    lastColor=color;ledEscrito=true;
    const uint8_t brillo=PanelConfig::brightness;
    neopixelWrite(PanelConfig::rgbPin, color==Color::Rojo || color==Color::Amarillo ? brillo : 0,
        color==Color::Verde || color==Color::Amarillo ? brillo : 0, color==Color::Azul ? brillo : 0);
}
}
// Contactor OFF antes de configurar el pin (evita un pulso al encender) y lectura inicial.
// Si START ya esta presionado al encender, se ignora hasta soltarlo.
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
// Energiza la bobina del contactor (a traves del TIP122): sierra ON.
void Panel::on() {
    digitalWrite(PanelConfig::contactorPin,PanelConfig::contactorActive);
    active=true;led();
    printReply("CONTACTOR ON (orden electrica)");
}
// Desenergiza la bobina del contactor: sierra OFF.
void Panel::off() {
    digitalWrite(PanelConfig::contactorPin,!PanelConfig::contactorActive);
    bool was=active;active=false;led();
    if (was) printReply("CONTACTOR OFF");
}
// Antirrebote de 30 ms por entrada e informe de cada cambio por el monitor.
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
// true si hay EMERGENCIA o STOP fisico activo: impide iniciar movimientos.
bool Panel::blocked() {
    const auto& stop=inputs[1];
    return emergencyActive() || (stop.enabled && (stop.stable || digitalRead(stop.pin)==stop.level));
}
bool Panel::emergencyActive() {
    const auto& emergency=inputs[2];
    return emergency.stable || digitalRead(emergency.pin)==emergency.level;
}
// true una sola vez por cada pulsacion de STOP.
bool Panel::nuevoStop() {
    const auto& stop=inputs[1];
    if (!stop.enabled) return false;
    if (!stop.raw && !stop.stable && uint32_t(millis()-stop.changedAt)>=PanelConfig::debounceMs)
        stopConsumido=false;
    if (!stop.raw || stopConsumido) return false;
    stopConsumido=true;
    return true;
}
// true una sola vez por pulsacion de START, solo si esta habilitado en la etapa
// actual y no hay bloqueo. Una pulsacion no habilitada se descarta hasta soltar.
bool Panel::nuevoStart(bool habilitado) {
    const auto& start=inputs[0];
    if (!start.raw && !start.stable && uint32_t(millis()-start.changedAt)>=PanelConfig::debounceMs)
        startConsumido=false;
    if (start.raw && (!habilitado || blocked())) startConsumido=true;
    if (!start.raw || !start.stable || startConsumido) return false;
    startConsumido=true;
    return true;
}
// Color y modo (fijo o titilando) que pide el coordinador para el LED.
void Panel::indicar(Color color, bool titila) { colorPedido=color;titilaPedido=titila;led(); }
// Estado del panel para el comando Q.
bool Panel::report() {
    char text[160];
    const auto& stop=inputs[1];
    const char* stopEstado=stop.enabled ? (stop.stable ? "1" : "0") : "DESHABILITADO";
    snprintf(text,sizeof(text),"PANEL: contactor_orden=%s START=%d STOP=%s EMERGENCIA=%d",
        active?"ON":"OFF",inputs[0].stable,stopEstado,inputs[2].stable);
    return printReply(text);
}
