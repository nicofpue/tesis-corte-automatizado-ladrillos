#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include "comandos.h"
#include "config_panel.h"
#include "config_secuencia.h"
#include "eje.h"
#include "geometria.h"
#include "motor3.h"
#include "panel.h"
#include "telemetria.h"

namespace {
using namespace SecuenciaConfig;

enum class Etapa {
    Inicializando, Referenciando, EsperandoStart, Sujetando, Posicionando,
    ArrancandoSierra, AvanzandoCorte, Retornando, ApagandoSierra,
    Liberando, AbriendoSinPieza, CicloCompletado, Pausado, Emergencia, PosicionamientoIncompleto, Bloqueado
};
enum class OrigenReinicio { Ninguno, Pulsador, Serie };
enum class Compresion { EsperandoContacto, Confirmando, Avanzando, Completa };

const AxisConfig cfg1 = motor1Config(), cfg2 = motor2Config(), cfg3 = motor3Config();
Axis m1(cfg1), m2(cfg2);
Axis* const ejes[] = {&m1, &m2};
const GeometryConfig geometria = geometryConfig();
Commands comandos;

Etapa etapa = Etapa::Inicializando;
OrigenReinicio origenReinicio = OrigenReinicio::Ninguno;
uint32_t primerReinicioMs = 0;
Etapa etapaPausada = Etapa::Inicializando;
uint32_t pausaDesde = 0;
bool pausaLista = false, homeSolicitado = false;
bool lista = false;
bool homeMordazasIniciado = false;
bool cancelado = false;
bool diagnosticoDisponible = false;
bool emergenciaDetectada = false;
bool derivaMarginal = false;
uint32_t derivaMarginalDesde = 0;
struct ConfirmacionHomeArranque {
    bool pendiente = false, activo = false;
    uint32_t desde = 0, cambioDesde = 0;
} homeArranque;
uint32_t previoUs = 0, encendidoDesde = 0, apagadoDesde = 0, listaDesde = 0;
float brechaSujeta = 0, brechaCentrada = 0;
float inicioTraslado[2] = {}, referencia[2] = {};
uint32_t inicioPosicionMs = 0, inicioEstabilizacionMs = 0;
int ejeEnPausa = -1;
bool estabilizando = false;
struct PausaContacto {
    bool activa = false, faltaContacto = false;
    uint32_t desde = 0, cambioDesde = 0;
} pausaContacto;
struct PerdidaContactoBanco {
    bool pendiente = false;
    uint32_t desde = 0;
};
PerdidaContactoBanco perdidasBanco[2];
char informePosicion[3][160];
struct Contacto {
    bool detenido = false, activo = false, confirmado = false;
    uint32_t detenidoDesde = 0, activoDesde = 0;
    Compresion compresion = Compresion::EsperandoContacto;
    float posicionContacto = 0;
    float objetivo = 0;
};
Contacto contactos[2];
const char* motivo = "";
char detalleCentrado[160];

const char* nombre() {
    switch (etapa) {
    case Etapa::Inicializando: return "INICIALIZANDO";
    case Etapa::Referenciando: return "REFERENCIANDO";
    case Etapa::EsperandoStart: return "ESPERANDO START";
    case Etapa::Sujetando: return "SUJETANDO";
    case Etapa::Posicionando: return "POSICIONANDO";
    case Etapa::ArrancandoSierra: return "ARRANCANDO SIERRA";
    case Etapa::AvanzandoCorte: return "AVANZANDO CORTE";
    case Etapa::Retornando: return "RETORNANDO";
    case Etapa::ApagandoSierra: return "APAGANDO SIERRA";
    case Etapa::Liberando: return "LIBERANDO";
    case Etapa::AbriendoSinPieza: return "ABRIENDO SIN PIEZA";
    case Etapa::CicloCompletado: return "CICLO COMPLETADO";
    case Etapa::Pausado: return "PAUSADO POR STOP";
    case Etapa::Emergencia: return "EMERGENCIA";
    case Etapa::PosicionamientoIncompleto: return "POSICIONAMIENTO INCOMPLETO";
    case Etapa::Bloqueado: return "BLOQUEADO";
    }
    return "?";
}

bool pasoListo(uint32_t ahora, uint32_t desde, uint32_t espera) {
    return uint32_t(ahora - desde) >= espera;
}

void entrar(Etapa siguiente) {
    etapa = siguiente;
    if (BANCO_SIN_SIERRA && (siguiente<=Etapa::Posicionando || siguiente>Etapa::ApagandoSierra)) {
        for (auto& perdida : perdidasBanco) perdida = {};
    }
    derivaMarginal = false;
    if (siguiente==Etapa::ArrancandoSierra) homeArranque = {};
    origenReinicio = OrigenReinicio::Ninguno;
    lista = false;
    previoUs = micros();
    printReply(nombre());
}

void completar(const char* mensaje) {
    if (lista) return;
    lista = true;
    listaDesde = millis();
    printReply(mensaje);
}

void detenerSalidas() {
    Panel::off();
    m1.stop();
    m2.stop();
    Motor3::stop();
}

void bloquear(const char* causa) {
    const bool falloEnCorte = etapa>=Etapa::ArrancandoSierra && etapa<=Etapa::ApagandoSierra;
    const char* fase = nombre();
    detenerSalidas();
    entrar(Etapa::Bloqueado);
    motivo = causa;
    printReply(causa);
    if (falloEnCorte) {
        char linea[160];
        snprintf(linea,sizeof(linea),"FALLO DE CICLO: etapa=%s; PWM=0; contactor orden OFF; CONTACTO M1=%d M2=%d",
            fase,m1.contactPressed(),m2.contactPressed());
        printReply(linea);
        Motor3::reportEssential();
    }
}

void bloquearMordaza(unsigned i, const char* causa) {
    snprintf(detalleCentrado, sizeof(detalleCentrado), "M%u: %s", i+1, causa);
    bloquear(detalleCentrado);
}

void confirmarReinicio(OrigenReinicio origen) {
    if (diagnosticoDisponible && Panel::blocked()) {
        origenReinicio = OrigenReinicio::Ninguno;
        printReply("Reinicio bloqueado por una entrada de parada activa.");
        return;
    }
    const uint32_t ahora = millis();
    if (origen!=origenReinicio || pasoListo(ahora, primerReinicioMs, CONFIRMACION_REINICIO_MS)) {
        origenReinicio = origen;
        primerReinicioMs = ahora;
        printReply(origen==OrigenReinicio::Pulsador ? "Reinicio: soltar y pulsar START otra vez."
                                                  : "Reinicio: enviar I otra vez.");
        return;
    }
    if (diagnosticoDisponible) detenerSalidas();
    ESP.restart();
}

bool homeM3() {
    return Motor3::isHomed() && Motor3::homePressed() && !Motor3::endPressed() && !Motor3::busy();
}

bool confirmarHomeDuranteArranque(uint32_t ahora) {
    if (!Motor3::isHomed() || Motor3::busy() || Motor3::endPressed()) {
        bloquear("M3 en estado incompatible con HOME durante el arranque."); return false;
    }
    const bool activo=Motor3::homePressed();
    if (!homeArranque.pendiente) {
        if (activo) return true;
        homeArranque = {true, false, ahora, ahora};
    } else if (activo!=homeArranque.activo) {
        homeArranque.activo = activo;
        homeArranque.cambioDesde = ahora;
    }
    const uint32_t confirmacionMs=activo ? cfg3.limitConfirmMs : HOME_ARRANQUE_PERDIDA_MS;
    if (pasoListo(ahora, homeArranque.cambioDesde, confirmacionMs)) {
        if (!activo) {
            bloquear("M3 perdio HOME durante el arranque (senal inactiva confirmada)."); return false;
        }
        homeArranque.pendiente = false;
        return true;
    }
    if (pasoListo(ahora, homeArranque.desde, HOME_ARRANQUE_ESTABILIZACION_MAX_MS))
        bloquear("Senal HOME de M3 inestable durante el arranque.");
    return false;
}

bool mordazasEnHome() {
    return m1.isHomed() && m2.isHomed() && m1.homePressed() && m2.homePressed()
        && !m1.endPressed() && !m2.endPressed() && !m1.busy() && !m2.busy();
}

bool algunContacto() { return m1.contactPressed() || m2.contactPressed(); }
float brecha() { return faceGap(geometria, m1.position(), m2.position()); }

void apagar() {
    Panel::off();
    apagadoDesde = millis();
}

void parar() {
    if (etapa==Etapa::Emergencia) return;
    if (etapa==Etapa::Pausado) {
        homeSolicitado = true;
        printReply("HOME solicitado. Esperar parada y confirmar con I.");
        return;
    }
    etapaPausada = etapa;
    pausaLista = lista;
    homeSolicitado = false;
    detenerSalidas();
    pausaDesde = millis();
    entrar(Etapa::Pausado);
    printReply("STOP: salidas OFF. START o I reanuda; otra orden STOP solicita HOME.");
}

bool pinesValidos() {
    const uint8_t pines[] = {
        cfg1.pwmPin, cfg1.dirPin, cfg1.encA, cfg1.encB, cfg1.homePin, cfg1.endPin, cfg1.contactPin,
        cfg2.pwmPin, cfg2.dirPin, cfg2.encA, cfg2.encB, cfg2.homePin, cfg2.endPin, cfg2.contactPin,
        cfg3.pwmPin, cfg3.dirPin, cfg3.encA, cfg3.encB, cfg3.homePin, cfg3.endPin,
        PanelConfig::contactorPin, PanelConfig::startPin, PanelConfig::stopPin,
        PanelConfig::emergencyPin, PanelConfig::rgbPin
    };
    for (size_t i=0; i<sizeof(pines); ++i) {
        if (pines[i]>=49) return false;
        for (size_t j=i+1; j<sizeof(pines); ++j)
            if (pines[i]==pines[j]) return false;
    }
    return cfg1.pwmChannel!=cfg2.pwmChannel && cfg1.pwmChannel!=cfg3.pwmChannel
        && cfg2.pwmChannel!=cfg3.pwmChannel;
}

void comenzarHome() {
    Panel::off();
    homeMordazasIniciado = false;
    entrar(Etapa::Referenciando);
    if (!Motor3::seekLimit(-1)) bloquear("No se pudo iniciar HOME de M3.");
}

void moverAReferencias() {
    estabilizando = false;
    ejeEnPausa = -1;
    pausaContacto = {};
    if (!m1.moveHeldTo(referencia[0])) { bloquear(m1.message()); return; }
    if (!m2.moveHeldTo(referencia[1])) bloquear(m2.message());
}

void abrirMordazas(Etapa siguiente) {
    if (!homeM3()) { bloquear("M3 fuera de HOME al abrir las mordazas."); return; }
    detenerSalidas();
    entrar(siguiente);
    if (!m1.home() || !m2.home()) bloquear("No se pudo iniciar HOME de las mordazas.");
}

bool posicionValida(float x1, float x2);

void reanudar() {
    switch (etapaPausada) {
    case Etapa::Inicializando:
    case Etapa::EsperandoStart:
    case Etapa::CicloCompletado:
        entrar(etapaPausada);
        lista = pausaLista;
        break;
    case Etapa::Referenciando:
        comenzarHome();
        break;
    case Etapa::Sujetando:
        if (!homeM3()) { bloquear("M3 fuera de HOME al reanudar sujecion."); return; }
        entrar(Etapa::Sujetando);
        for (unsigned i=0; i<2; ++i) {
            Axis& eje = *ejes[i];
            if (contactos[i].detenido && !eje.contactPressed()) {
                bloquear("CONTACTO perdido durante la pausa."); return;
            }
            Contacto& contacto = contactos[i];
            if (BANCO_SIN_SIERRA && contacto.detenido) {
                // Conservar el objetivo original para que STOP no agregue otro avance.
                contacto.activo = false;
                contacto.confirmado = false;
                contacto.detenidoDesde = millis();
                if (contacto.compresion==Compresion::Avanzando && !eje.compressTo(contacto.objetivo)) {
                    bloquearMordaza(i, eje.message());
                    return;
                }
                continue;
            }
            contactos[i] = {};
            if (!eje.contactPressed() && !eje.approachTo(eje.config().maxTravelMm)) {
                bloquear(eje.message()); return;
            }
        }
        break;
    case Etapa::Posicionando:
    case Etapa::ArrancandoSierra:
        if (!homeM3() || !m1.contactPressed() || !m2.contactPressed()) {
            bloquear("Revisar HOME de M3 y ambos CONTACTOS."); return;
        }
        if (etapaPausada==Etapa::ArrancandoSierra && !pasoListo(millis(), pausaDesde, PARADA_MS)) {
            printReply("Esperar parada de la sierra antes de reanudar."); return;
        }
        entrar(Etapa::Posicionando);
        if ((pausaLista || etapaPausada==Etapa::ArrancandoSierra) && posicionValida(m1.position(),m2.position())) {
            completar("LISTO: centrado confirmado para reanudar el ciclo.");
        } else {
            if (pausaLista || etapaPausada==Etapa::ArrancandoSierra) inicioPosicionMs = millis();
            else inicioPosicionMs += millis()-pausaDesde;
            moverAReferencias();
        }
        break;
    case Etapa::ApagandoSierra:
        entrar(Etapa::ApagandoSierra);
        break;
    case Etapa::Liberando:
    case Etapa::AbriendoSinPieza:
        abrirMordazas(etapaPausada);
        break;
    default:
        printReply("Corte o retorno interrumpido. Otro STOP solicita HOME con la sierra OFF.");
        break;
    }
}

uint32_t esperaRecuperacionHome() {
    return etapa==Etapa::Emergencia ? ESPERA_EMERGENCIA_MS : PARADA_MS;
}

bool startAutorizaHome() {
    return lista && (etapa==Etapa::Inicializando || etapa==Etapa::Emergencia);
}

void iniciar() {
    if (etapa==Etapa::Pausado) {
        if (!homeSolicitado) { reanudar(); return; }
        if (!pasoListo(millis(), pausaDesde, PARADA_MS)) {
            printReply("Esperar parada de la sierra antes de confirmar HOME."); return;
        }
        comenzarHome();
        return;
    }
    if (etapa==Etapa::Inicializando || etapa==Etapa::Emergencia) {
        if (!lista) { printReply("Esperar parada y restablecer el circuito fisico antes de HOME."); return; }
        comenzarHome();
    } else if (etapa==Etapa::EsperandoStart || etapa==Etapa::CicloCompletado) {
        if (!mordazasEnHome() || !homeM3() || algunContacto()) {
            bloquear("START: revisar los tres HOME y CONTACTOS en reposo."); return;
        }
        cancelado = false;
        for (auto& contacto : contactos) contacto = {};
        entrar(Etapa::Sujetando);
        if (!m1.approachTo(cfg1.maxTravelMm) || !m2.approachTo(cfg2.maxTravelMm))
            bloquear("No se pudo iniciar cierre de las mordazas.");
    } else {
        printReply("I autoriza HOME al encender o recuperar, e inicia el ciclo cuando esta listo.");
    }
}

void informarApertura(const char* estado, float x1, float x2) {
    const auto pieza = estimatedGrip(geometria, x1, x2);
    char linea[160];
    snprintf(linea,sizeof(linea),"%s: apertura EST=%.3f mm; separacion REF=%.3f mm; centro EST=%+.3f mm (+derecha)",
        estado,pieza.width,faceGap(geometria,x1,x2),pieza.center);
    printReply(linea);
}

void iniciarPosicionamiento() {
    if (!m1.contactPressed() || !m2.contactPressed()) {
        bloquear("Se requieren ambos CONTACTOS para centrar."); return;
    }
    inicioTraslado[0] = m1.position();
    inicioTraslado[1] = m2.position();
    const auto pieza = estimatedGrip(geometria, inicioTraslado[0], inicioTraslado[1]);
    if (!isfinite(pieza.center) || !isfinite(pieza.width) || pieza.width<=0) {
        bloquear("Centrado rechazado: geometria invalida."); return;
    }
    const float traslado = fabsf(pieza.center)<=geometria.centerToleranceMm ? 0.0f : -pieza.center;
    const float ajusteM1 = BANCO_SIN_SIERRA ? BANCO_AJUSTE_CENTRADO_M1_MM : 0.0f;
    const float objetivoM1 = inicioTraslado[0]+traslado+ajusteM1;
    const float ajusteM2 = BANCO_SIN_SIERRA ? BANCO_AJUSTE_CENTRADO_M2_MM : 0.0f;
    const float objetivoM2 = inicioTraslado[1]-traslado+ajusteM2;
    ShiftRange recorrido;
    if (!translationRange(inicioTraslado[0], inicioTraslado[1], cfg1.maxTravelMm, cfg2.maxTravelMm, recorrido)
        || objetivoM1<0 || objetivoM1>cfg1.maxTravelMm || objetivoM2<0 || objetivoM2>cfg2.maxTravelMm
        || (objetivoM1>inicioTraslado[0] && m1.endPressed())
        || (objetivoM1<inicioTraslado[0] && m1.homePressed())
        || (objetivoM2>inicioTraslado[1] && m2.endPressed())
        || (objetivoM2<inicioTraslado[1] && m2.homePressed())) {
        snprintf(detalleCentrado,sizeof(detalleCentrado),
            "Centrado rechazado: traslado de %+.3f mm fuera de recorrido o bloqueado por un final.",traslado);
        bloquear(detalleCentrado);return;
    }
    referencia[0] = objetivoM1;
    referencia[1] = objetivoM2;
    brechaSujeta = faceGap(geometria, inicioTraslado[0], inicioTraslado[1]);
    entrar(Etapa::Posicionando);
    char linea[180];
    snprintf(linea,sizeof(linea),"CENTRADO EST: traslado=%+.3f mm (+derecha); ajuste M1=%+.3f M2=%+.3f mm; objetivos M1=%.3f M2=%.3f; separacion REF inicial=%.3f mm",
        traslado,ajusteM1,ajusteM2,referencia[0],referencia[1],brechaSujeta);
    printReply(linea);
    if (traslado==0.0f && ajusteM1==0.0f && ajusteM2==0.0f) {
        brechaCentrada = brechaSujeta;
        informarApertura("CENTRADO",inicioTraslado[0],inicioTraslado[1]);
        completar("LISTO: centro estimado dentro de tolerancia.");
        return;
    }
    inicioPosicionMs = millis();
    moverAReferencias();
}

bool arranqueManual() {
    return HABILITAR_CONTACTOR_REAL && ARRANQUE_REAL_MS==0;
}

bool aperturaAutomatica() {
    return !HABILITAR_CONTACTOR_REAL || PARADA_REAL_VERIFICADA_MS>0;
}

void avanzar() {
    if (derivaMarginal) { printReply("Esperar estabilizacion de la pieza."); return; }
    if (etapa==Etapa::Pausado) {
        printReply("Ciclo pausado. START o I reanuda; otro STOP solicita HOME."); return;
    }
    if (etapa==Etapa::Inicializando || etapa==Etapa::Emergencia || etapa==Etapa::EsperandoStart) {
        printReply("En esta etapa corresponde I. Consultar Q."); return;
    }
    if (!lista && !(etapa==Etapa::ArrancandoSierra && arranqueManual())) {
        printReply("Etapa en curso. Consultar Q y esperar LISTO."); return;
    }
    switch (etapa) {
    case Etapa::Referenciando:
    case Etapa::CicloCompletado:
        if (!mordazasEnHome() || !homeM3()) {
            bloquear("Revisar los tres HOME."); return;
        }
        if (algunContacto()) { printReply("Retirar la pieza y comprobar CONTACTOS antes de N."); return; }
        entrar(Etapa::EsperandoStart);
        printReply("Preparar pieza y pulsar START o enviar I.");
        break;
    case Etapa::Sujetando:
        iniciarPosicionamiento();
        break;
    case Etapa::Posicionando:
        if (BANCO_SIN_SIERRA && (!m1.contactPressed() || !m2.contactPressed())) {
            bloquear("Se requieren ambos CONTACTOS para accionar el contactor."); return;
        }
        entrar(Etapa::ArrancandoSierra);
        if (HABILITAR_CONTACTOR_REAL) Panel::on();
        else printReply("Arranque simulado: contactor permanece OFF.");
        encendidoDesde = millis();
        if (arranqueManual())
            printReply("ARRANQUE MANUAL: enviar N solo al verificar que el disco alcanzo su velocidad de trabajo.");
        {
            char linea[140];
            snprintf(linea,sizeof(linea),"M3: PWM objetivo %.1f%%; rampa %lu ms; proteccion sin cambio de encoder %lu ms",
                cfg3.endSeekPwm,static_cast<unsigned long>(M3_SEEK_RAMP_MS),
                static_cast<unsigned long>(cfg3.noCountsTimeoutMs));
            printReply(linea);
        }
        break;
    case Etapa::ArrancandoSierra:
        if (homeArranque.pendiente || !homeM3()) {
            printReply("Avance inhibido: esperar confirmacion estable de HOME."); return;
        }
        entrar(Etapa::AvanzandoCorte);
        if (!Motor3::seekLimit(1)) bloquear("No se pudo iniciar avance a END de M3.");
        break;
    case Etapa::AvanzandoCorte:
        entrar(Etapa::Retornando);
        if (!Motor3::seekLimit(-1)) bloquear("No se pudo iniciar retorno de M3.");
        break;
    case Etapa::ApagandoSierra:
        abrirMordazas(Etapa::Liberando);
        break;
    case Etapa::Liberando:
        entrar(Etapa::CicloCompletado);
        completar(cancelado ? "Corte cancelado. Retirar pieza antes de otro START."
                            : "Retirar pieza. Una nueva orden START inicia otro ciclo.");
        break;
    default:
        printReply("En esta etapa corresponde I. Consultar Q.");
        break;
    }
}

void continuarCiclo() {
    if (!lista || Panel::blocked() || derivaMarginal
        || (etapa==Etapa::ArrancandoSierra && homeArranque.pendiente)) return;
    switch (etapa) {
    case Etapa::Referenciando:
    case Etapa::Sujetando:
    case Etapa::Posicionando:
    case Etapa::ArrancandoSierra:
    case Etapa::AvanzandoCorte:
    case Etapa::Liberando:
        avanzar();
        break;
    case Etapa::ApagandoSierra:
        if (aperturaAutomatica()) avanzar();
        break;
    default:
        break;
    }
}

void cierreSinSujecion() {
    if (algunContacto() || contactos[0].detenido || contactos[1].detenido
        || m1.state()==AxisState::ContactLimit || m2.state()==AxisState::ContactLimit) {
        bloquear("Recorrido agotado con contacto parcial. Revisar la pieza."); return;
    }
    printReply("SIN PIEZA: recorrido agotado sin CONTACTO. Mordazas a HOME.");
    abrirMordazas(Etapa::AbriendoSinPieza);
}

// Cada mordaza confirma su contacto y completa su propia compresion.
void atenderCompresion(unsigned i, uint32_t ahora) {
    Axis& eje = *ejes[i];
    Contacto& contacto = contactos[i];

    if (contacto.compresion==Compresion::Confirmando && contacto.confirmado) {
        if (contacto.objetivo<=contacto.posicionContacto) {
            bloquearMordaza(i, "Sin recorrido para comprimir despues de CONTACTO.");
            return;
        }
        if (!eje.compressTo(contacto.objetivo)) {
            bloquearMordaza(i, eje.message());
            return;
        }
        contacto.compresion = Compresion::Avanzando;
        char linea[120];
        snprintf(linea, sizeof(linea), "M%u COMPRESION: objetivo=%.3f mm; avance=%.3f mm; PWM=%.1f%%",
                 i+1, contacto.objetivo, contacto.objetivo-contacto.posicionContacto,
                 eje.config().compressionPwm);
        printReply(linea);
        return;
    }
    if (contacto.compresion!=Compresion::Avanzando || eje.busy()) return;
    if (eje.state()!=AxisState::Reached) {
        bloquearMordaza(i, "Se detuvo antes de completar la compresion.");
        return;
    }

    contacto.compresion = Compresion::Completa;
    contacto.detenidoDesde = ahora;
    char linea[120];
    snprintf(linea, sizeof(linea), "M%u COMPRESION: %s; avance medido=%.3f de %.3f mm; PWM=0.",
             i+1, eje.message(), eje.position()-contacto.posicionContacto,
             contacto.objetivo-contacto.posicionContacto);
    printReply(linea);
}

void atenderCierre(uint32_t ahora) {
    bool ambos = true;
    for (unsigned i=0; i<2; ++i) {
        Axis& eje = *ejes[i];
        Contacto& contacto = contactos[i];
        if (eje.endPressed() || eje.state()==AxisState::EndLimit) { cierreSinSujecion(); return; }
        const bool activo = eje.contactPressed();
        if (!contacto.detenido && (activo || eje.state()==AxisState::ContactLimit)) {
            eje.haltContact();
            contacto.detenido = true;
            contacto.detenidoDesde = ahora;
            if (BANCO_SIN_SIERRA) {
                contacto.posicionContacto = eje.position();
                // Acortar el avance cuando el contacto queda cerca del final del recorrido.
                contacto.objetivo = fminf(contacto.posicionContacto+BANCO_COMPRESION_MM,
                                          eje.config().maxTravelMm);
                contacto.compresion = Compresion::Confirmando;
            }
            printReply(i==0 ? "M1 detenido por CONTACTO." : "M2 detenido por CONTACTO.");
        }
        if (!contacto.detenido && !eje.busy()) {
            if (eje.state()==AxisState::Reached) cierreSinSujecion();
            else bloquear("Una mordaza se detuvo antes de confirmar CONTACTO.");
            return;
        }
        if (activo && !contacto.activo) contacto.activoDesde = ahora;
        contacto.activo = activo;
        const bool confirmado = activo && pasoListo(ahora, contacto.activoDesde, eje.config().limitConfirmMs);
        if (!BANCO_SIN_SIERRA && ((!activo && contacto.confirmado) || (contacto.detenido && !confirmado
            && pasoListo(ahora, contacto.detenidoDesde, eje.config().homeConfirmTimeoutMs)))) {
            bloquear("CONTACTO perdido o inestable durante la sujecion."); return;
        }
        contacto.confirmado = confirmado;
        if (BANCO_SIN_SIERRA) {
            atenderCompresion(i, ahora);
            if (etapa==Etapa::Bloqueado) return;
            ambos &= contacto.compresion==Compresion::Completa;
        }
        ambos &= confirmado;
    }
    if (ambos && !lista && pasoListo(ahora, contactos[0].detenidoDesde, geometria.contactSettlingMs)
        && pasoListo(ahora, contactos[1].detenidoDesde, geometria.contactSettlingMs)) {
        informarApertura("SUJETA",m1.position(),m2.position());
        completar("LISTO: contactos confirmados y estabilizados. Centrando a continuacion.");
    }
}

bool posicionValida(float x1, float x2) {
    return fabsf(x1-referencia[0])<=TOLERANCIA_CENTRADO_MM
        && fabsf(x2-referencia[1])<=TOLERANCIA_CENTRADO_MM;
}

void informarPosicionIncompleta() {
    for (const auto& fila : informePosicion) printReply(fila);
}

void terminarPorPlazo(uint32_t transcurrido) {
    const float brechaFinal = brecha();
    for (unsigned i=0; i<2; ++i) {
        const Axis& eje = *ejes[i];
        const int32_t cuentas = eje.counts();
        const float x = static_cast<float>(cuentas)*eje.config().encoderSign*eje.config().mmPerCount();
        snprintf(informePosicion[i], sizeof(informePosicion[i]),
            "M%u: inicio=%.3f x=%.3f objetivo=%.3f falta=%+.3f mm; PWM previo=%.2f%% CONTACTO=%d cuentas=%ld",
            i+1, inicioTraslado[i], x, referencia[i], referencia[i]-x,
            eje.pwm(), eje.contactPressed(), static_cast<long>(cuentas));
    }
    detenerSalidas();
    entrar(Etapa::PosicionamientoIncompleto);
    snprintf(informePosicion[2], sizeof(informePosicion[2]),
        "Plazo: %lu/%lu ms; separacion REF inicio=%.3f final=%.3f cambio=%+.3f mm; PWM=0",
        static_cast<unsigned long>(transcurrido), static_cast<unsigned long>(geometria.heldTimeoutMs),
        brechaSujeta, brechaFinal, brechaFinal-brechaSujeta);
    informarPosicionIncompleta();
}

bool contactosDeCentrado(uint32_t ahora) {
    const bool falta = !m1.contactPressed() || !m2.contactPressed();
    if (!falta && !pausaContacto.activa) return true;
    for (Axis* eje : ejes) eje->pauseHeldMotion();
    estabilizando = false;
    if (!pausaContacto.activa) {
        pausaContacto = {true, falta, ahora, ahora};
    } else if (pausaContacto.faltaContacto!=falta) {
        pausaContacto.faltaContacto = falta;
        pausaContacto.cambioDesde = ahora;
    }
    if (falta && pasoListo(ahora, pausaContacto.cambioDesde, geometria.contactLossConfirmMs)) {
        bloquear("CONTACTO perdido durante centrado."); return false;
    }
    if (!falta && pasoListo(ahora, pausaContacto.cambioDesde, max(cfg1.limitConfirmMs,cfg2.limitConfirmMs))) {
        pausaContacto.activa = false;
        return true;
    }
    if (pasoListo(ahora, pausaContacto.desde, geometria.contactRecoverTimeoutMs))
        bloquear("CONTACTOS no se estabilizan durante centrado.");
    return false;
}

void atenderPosicionamiento(uint32_t ahora) {
    if (lista) return;
    for (const Axis* eje : ejes) {
        if (!eje->busy() && eje->state()!=AxisState::Reached) {
            bloquear("Un sensor impidio llegar a la referencia de mordaza."); return;
        }
    }
    if (!BANCO_SIN_SIERRA) {
        const uint32_t transcurrido = ahora-inicioPosicionMs;
        if (transcurrido>=geometria.heldTimeoutMs) { terminarPorPlazo(transcurrido); return; }
        if (!contactosDeCentrado(ahora)) return;
    }
    if (!m1.busy() && !m2.busy()) {
        ejeEnPausa = -1;
        if (!BANCO_SIN_SIERRA) {
            if (!estabilizando) { inicioEstabilizacionMs = ahora; estabilizando = true; }
            if (!pasoListo(ahora, inicioEstabilizacionMs, ESTABILIZACION_FINAL_CENTRADO_MS)) return;
        }
        const float x1=m1.position(), x2=m2.position();
        for (size_t i=0; i<2; ++i) {
            if (ejes[i]->stalledHeld()) {
                char aviso[120];
                snprintf(aviso,sizeof(aviso),"M%u se detuvo sin avance durante el centrado; error=%+.3f mm (lim %.3f). Se evalua el centrado.",
                    static_cast<unsigned>(i+1), referencia[i]-ejes[i]->position(), TOLERANCIA_CENTRADO_MM);
                printReply(aviso);
            }
        }
        const float separacion=faceGap(geometria,x1,x2);
        // Comparar contra la separacion que dejan los objetivos, incluidos los ajustes de las mordazas.
        const float brechaObjetivo = faceGap(geometria, referencia[0], referencia[1]);
        const float cambio = separacion-brechaObjetivo;
        const float limiteSeparacion = BANCO_SIN_SIERRA ? BANCO_CAMBIO_SEPARACION_MM : geometria.maxHeldGapChangeMm;
        if (!posicionValida(x1,x2) || fabsf(cambio)>limiteSeparacion) {
            snprintf(detalleCentrado,sizeof(detalleCentrado),"Centrado fuera de tolerancia: error M1=%+.3f M2=%+.3f (lim %.3f); error separacion vs objetivo=%+.3f (lim %.3f) mm.",
                referencia[0]-x1,referencia[1]-x2,TOLERANCIA_CENTRADO_MM,
                cambio,limiteSeparacion);
            bloquear(detalleCentrado);
            return;
        }
        brechaCentrada = separacion;
        informarApertura("CENTRADO",x1,x2);
        completar(BANCO_SIN_SIERRA ? "LISTO: referencias de banco OK." : "LISTO: referencias y CONTACTOS OK.");
        return;
    }
    // Misma compensacion de desfase que el traslado C/T del proyecto base.
    const float deltaM1 = referencia[0]-inicioTraslado[0];
    const float deltaM2 = referencia[1]-inicioTraslado[1];
    if (deltaM1*deltaM2>=0) return; // El ajuste de sincronismo requiere sentidos opuestos.
    const int8_t sentido = deltaM1>0 ? 1 : -1;
    const float avanceM2 = m2.position()-inicioTraslado[1];
    const float fraccion = constrain(avanceM2/deltaM2, 0.0f, 1.0f);
    const float error = (m1.position()-inicioTraslado[0]+avanceM2-(deltaM1+deltaM2)*fraccion)*sentido;
    const int adelantado = error>0 ? 0 : 1;
    const float umbral = ejeEnPausa==adelantado ? geometria.maxHeldGapChangeMm : geometria.heldSyncPauseMm;
    ejeEnPausa = -1;
    Axis& eje = *ejes[adelantado];
    const int8_t sentidoEje = adelantado==0 ? sentido : -sentido;
    if (!BANCO_SIN_SIERRA && fabsf(error)>umbral && eje.busy() && ejes[1-adelantado]->busy()
        && (referencia[adelantado]-eje.position())*sentidoEje>TOLERANCIA_CENTRADO_MM) {
        ejeEnPausa = adelantado;
        eje.pauseHeldMotion();
    }
    const float ajuste = constrain(geometria.heldSyncGainPwmPerMm*error,
                                   -geometria.maxHeldSyncPwm, geometria.maxHeldSyncPwm);
    m1.synchronization(-ajuste, sentido);
    m2.synchronization(ajuste, -sentido);
}

uint32_t esperaArranque() {
    return HABILITAR_CONTACTOR_REAL ? ARRANQUE_REAL_MS : ARRANQUE_SIMULADO_MS;
}

bool confirmarContactosSujetos(uint32_t ahora) {
    if (!BANCO_SIN_SIERRA) {
        if (m1.contactPressed() && m2.contactPressed()) return true;
        bloquear("Perdida de CONTACTO con pieza sujeta.");
        return false;
    }

    const bool activo[] = {m1.contactPressed(), m2.contactPressed()};
    uint32_t inactivoMs[2] = {};
    for (unsigned i=0; i<2; ++i) {
        auto& perdida = perdidasBanco[i];
        if (activo[i]) perdida = {};
        else {
            if (!perdida.pendiente) perdida = {true, ahora};
            inactivoMs[i] = ahora-perdida.desde;
        }
    }
    if (inactivoMs[0]<BANCO_PERDIDA_CONTACTO_MS && inactivoMs[1]<BANCO_PERDIDA_CONTACTO_MS) return true;
    snprintf(detalleCentrado,sizeof(detalleCentrado),
        "Perdida de CONTACTO en banco: M1=%d (%lu ms inactivo), M2=%d (%lu ms inactivo); limite=%lu ms.",
        activo[0], static_cast<unsigned long>(inactivoMs[0]), activo[1],
        static_cast<unsigned long>(inactivoMs[1]), static_cast<unsigned long>(BANCO_PERDIDA_CONTACTO_MS));
    bloquear(detalleCentrado);
    return false;
}

void supervisar(bool stopPedido=false) {
    if (diagnosticoDisponible && Panel::emergencyActive()) {
        if (!emergenciaDetectada) {
            detenerSalidas();
            apagadoDesde = millis();
            entrar(Etapa::Emergencia);
            printReply("Cuenta de parada iniciada al detectar EMERGENCIA. Restablecer el circuito antes de autorizar HOME.");
        }
        emergenciaDetectada = true;
        lista = false;
        return;
    }
    emergenciaDetectada = false;
    if (etapa==Etapa::Bloqueado || etapa==Etapa::PosicionamientoIncompleto) return;
    if (stopPedido) parar();
    if (etapa==Etapa::Bloqueado) return;
    if (m1.hasFault()) { bloquearMordaza(0, m1.message()); return; }
    if (m2.hasFault()) { bloquearMordaza(1, m2.message()); return; }
    if (Motor3::hasFault()) { bloquear(Motor3::message()); return; }
    if ((m1.homePressed() && m1.endPressed()) || (m2.homePressed() && m2.endPressed())
        || (Motor3::homePressed() && Motor3::endPressed())) {
        bloquear("HOME y END activos al mismo tiempo en un eje."); return;
    }
    const uint32_t ahora = millis();
    const bool sujeta = etapa>=Etapa::Posicionando && etapa<=Etapa::ApagandoSierra;
    const bool centrando = etapa==Etapa::Posicionando && !lista;
    if (sujeta && !centrando && !confirmarContactosSujetos(ahora)) return;
    if (HABILITAR_CONTACTOR_REAL && !BANCO_SIN_SIERRA && sujeta && !centrando) {
        const float x1=m1.position(), x2=m2.position();
        const float error1=referencia[0]-x1, error2=referencia[1]-x2;
        // Los ejes cierran en sentidos opuestos: su diferencia da el cambio de centro.
        const float derivaCentro=(error2-error1)*0.5f;
        const float separacion=faceGap(geometria,x1,x2);
        const float cambio=separacion-brechaCentrada;
        // Absorber solo el redondeo de float al restar posiciones; una cuenta vale 0.008 mm.
        constexpr float redondeoMm=0.0001f;
        constexpr float limiteCentro=DERIVA_CENTRO_MM+DERIVA_MARGEN_VIBRACION_MM;
        constexpr float limiteSeparacion=DERIVA_SEPARACION_MM+DERIVA_MARGEN_VIBRACION_MM;
        const float exceso=fmaxf(fabsf(derivaCentro)-limiteCentro,fabsf(cambio)-limiteSeparacion)-redondeoMm;
        if (exceso<=0) derivaMarginal = false;
        else if (!derivaMarginal) { derivaMarginal = true; derivaMarginalDesde = ahora; }
        if (exceso>DERIVA_BANDA_TRANSITORIA_MM
            || (derivaMarginal && pasoListo(ahora, derivaMarginalDesde, DERIVA_CONFIRMACION_MS))) {
            snprintf(detalleCentrado,sizeof(detalleCentrado),
                "Deriva pieza: centro=%+.6f (lim %.3f), separacion=%+.6f (lim %.3f) mm; REF centrada=%.3f actual=%.3f; M1=%+.6f M2=%+.6f.",
                derivaCentro,limiteCentro,cambio,limiteSeparacion,brechaCentrada,separacion,error1,error2);
            bloquear(detalleCentrado); return;
        }
    }

    switch (etapa) {
    case Etapa::Inicializando:
    case Etapa::Emergencia:
        if (!Panel::blocked() && pasoListo(ahora, apagadoDesde, esperaRecuperacionHome()))
            completar("LISTO: verificar disco inmovil y recorrido libre; pulsar START o enviar I para HOME.");
        break;
    case Etapa::Referenciando:
        if (Motor3::busy()) break;
        if (!homeM3()) { bloquear("M3 no confirma HOME."); break; }
        if (!homeMordazasIniciado) {
            homeMordazasIniciado = true;
            if (!m1.home() || !m2.home()) bloquear("No se pudo iniciar HOME de mordazas.");
        } else if (mordazasEnHome()) completar("LISTO: HOME de los tres ejes.");
        else if (!m1.busy() && !m2.busy()) bloquear("HOME de mordazas no confirmado.");
        break;
    case Etapa::Sujetando:
        if (!homeM3()) bloquear("M3 salio de HOME durante la sujecion.");
        else atenderCierre(ahora);
        break;
    case Etapa::Posicionando:
        if (!homeM3()) bloquear("M3 salio de HOME durante el posicionamiento.");
        else atenderPosicionamiento(ahora);
        break;
    case Etapa::ArrancandoSierra:
        if (!confirmarHomeDuranteArranque(ahora)) break;
        if ((arranqueManual() || lista)
            && pasoListo(ahora, arranqueManual() ? encendidoDesde : listaDesde, CONFIRMACION_CON_SIERRA_MS)) {
            apagar();
            cancelado = true;
            entrar(Etapa::ApagandoSierra);
            printReply("Confirmacion de corte vencida. Sierra OFF; esperar antes de liberar.");
        }
        else if (!arranqueManual() && pasoListo(ahora, encendidoDesde, esperaArranque()))
            completar("LISTO: tiempo de arranque cumplido.");
        break;
    case Etapa::AvanzandoCorte:
        if (Motor3::busy()) break;
        if (!Motor3::endPressed() || Motor3::homePressed()) bloquear("M3 no confirma END.");
        else if (lista && pasoListo(ahora, listaDesde, CONFIRMACION_CON_SIERRA_MS))
            bloquear("Espera de N en END excedida. Sierra OFF. Revisar M3 y pieza.");
        else completar("LISTO: M3 en END. Retorno con la pieza sujeta.");
        break;
    case Etapa::Retornando:
        if (Motor3::busy()) break;
        if (!homeM3()) bloquear("M3 no confirma retorno a HOME.");
        else {
            apagar();
            entrar(Etapa::ApagandoSierra);
        }
        break;
    case Etapa::ApagandoSierra:
        if (!homeM3()) bloquear("M3 salio de HOME durante la parada.");
        else if (pasoListo(ahora, apagadoDesde, APERTURA_TRAS_PARADA_MS))
            completar(aperturaAutomatica() ? "LISTO: tiempo de parada cumplido. Apertura automatica."
                                          : "LISTO: verificar disco inmovil y enviar N para abrir.");
        break;
    case Etapa::Liberando:
    case Etapa::AbriendoSinPieza:
        if (!homeM3()) bloquear("M3 salio de HOME durante la liberacion.");
        else if (!m1.busy() && !m2.busy()) {
            if (!mordazasEnHome() || algunContacto()) bloquear("Liberacion: revisar HOME y CONTACTO.");
            else if (etapa==Etapa::AbriendoSinPieza) {
                entrar(Etapa::EsperandoStart);
                printReply("LISTO: mordazas en HOME. Pulsar START o enviar I para otro intento.");
            } else completar("LISTO: mordazas abiertas.");
        }
        break;
    default:
        break;
    }
}

void consultar(bool diagnostico) {
    char linea[160];
    snprintf(linea, sizeof(linea), "%s | %s | CONTACTOR %s | STOP/EMERG=%d",
             nombre(), lista ? "LISTO" : "EN ESPERA", BANCO_SIN_SIERRA ? "REAL (BANCO SIN SIERRA)" : (HABILITAR_CONTACTOR_REAL ? "REAL" : "SIMULADO"),
             Panel::blocked());
    printReply(linea);
    if (etapa==Etapa::Bloqueado) printReply(motivo);
    if (etapa==Etapa::Pausado) {
        printReply(homeSolicitado ? "HOME solicitado: confirmar con I tras la espera."
                                 : "Pausa: START o I reanuda; otra orden STOP solicita HOME.");
        if (homeSolicitado || etapaPausada==Etapa::ArrancandoSierra) {
            const uint32_t transcurrido = millis()-pausaDesde;
            snprintf(linea,sizeof(linea),"Espera de parada restante: %lu ms",
                static_cast<unsigned long>(transcurrido>=PARADA_MS ? 0 : PARADA_MS-transcurrido));
            printReply(linea);
        }
    }
    if (etapa==Etapa::PosicionamientoIncompleto) {
        informarPosicionIncompleta();
        if (!diagnostico) { Panel::report(); return; }
    }
    if (etapa==Etapa::ApagandoSierra || etapa==Etapa::Inicializando || etapa==Etapa::Emergencia) {
        const uint32_t transcurrido = millis()-apagadoDesde;
        const uint32_t espera = etapa==Etapa::ApagandoSierra ? APERTURA_TRAS_PARADA_MS : esperaRecuperacionHome();
        snprintf(linea, sizeof(linea), "Espera de parada restante: %lu ms",
                 static_cast<unsigned long>(transcurrido>=espera ? 0 : espera-transcurrido));
        printReply(linea);
    }
    for (unsigned i=0; i<2; ++i) {
        printReply(i==0 ? "M1" : "M2");
        printStatus(*ejes[i]);
        if (diagnostico) {
            printPins(*ejes[i]);
            printEncoderDiagnostic(ejes[i]->encoderDiagnostic(), ejes[i]->config());
        }
    }
    Motor3::reportEssential();
    if (Motor3::hasFault()) Motor3::reportFault();
    if (diagnostico) { Motor3::reportPins(); Motor3::reportEncoder(); }
    Panel::report();
}

void ayuda() {
    if (BANCO_SIN_SIERRA)
        printReply("BANCO SIN SIERRA: contactor real. Mantener la sierra desconectada.");
    printReply("I = HOME inicial o recuperacion; inicia ciclo o reanuda STOP segun etapa");
    printReply("START inicia un ciclo completo; N se reserva para confirmaciones manuales.");
    printReply("S o X = STOP | Q = estado | E = diagnostico | ? = ayuda");
    printReply("START fisico inicia el ciclo en ESPERANDO START o CICLO COMPLETADO, o reanuda una pausa.");
    printReply(PanelConfig::physicalStopEnabled ? "START fisico reanuda tras soltar STOP."
        : "STOP fisico deshabilitado. Usar S/X para pausar o solicitar HOME.");
    printReply("EMERGENCIA: restablecer circuito; tras LISTO, START o I autoriza HOME.");
    printReply("BLOQUEADO: dos START o dos I separados para reiniciar.");
}

void actualizarIndicador() {
    using Panel::Indicador;
    if (!PanelConfig::stageIndicatorsEnabled) return;
    if (etapa==Etapa::Bloqueado || etapa==Etapa::PosicionamientoIncompleto)
        Panel::indicar(Indicador::Bloqueado);
    else if (etapa==Etapa::Pausado || Panel::blocked()) Panel::indicar(Indicador::Espera);
    else if (m1.busy() || m2.busy() || Motor3::busy()) Panel::indicar(Indicador::Movimiento);
    else Panel::indicar(lista || etapa==Etapa::EsperandoStart ? Indicador::Listo : Indicador::Espera);
}
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    if (!pinesValidos()) {
        etapa = Etapa::Bloqueado;
        motivo = "Conflicto de pines o canales. Revisar config_eje.h y config_panel.h.";
        printReply(motivo);
        return;
    }
    Panel::begin();
    apagadoDesde = millis();
    if (!isfinite(geometria.contactOffsetMm)
        || !isfinite(geometria.leftContactShare) || geometria.leftContactShare<0 || geometria.leftContactShare>1
        || !isfinite(geometria.leftHomeToCenterMm) || geometria.leftHomeToCenterMm<=0
        || !isfinite(geometria.rightHomeToCenterMm) || geometria.rightHomeToCenterMm<=0
        || !isfinite(geometria.maxHeldGapChangeMm) || geometria.maxHeldGapChangeMm<=0
        || !isfinite(geometria.centerToleranceMm) || geometria.centerToleranceMm<0
        || !isfinite(geometria.heldSyncPauseMm) || geometria.heldSyncPauseMm<geometria.maxHeldGapChangeMm
        || geometria.contactLossConfirmMs==0 || geometria.contactRecoverTimeoutMs<geometria.contactLossConfirmMs
        || geometria.heldTimeoutMs<=geometria.contactSettlingMs
        || geometria.heldTimeoutMs<=ESTABILIZACION_FINAL_CENTRADO_MS
        || geometria.heldTimeoutMs>min(cfg1.moveTimeoutMs,cfg2.moveTimeoutMs)) {
        bloquear("Configuracion de posicionamiento invalida."); return;
    }
    const AxisConfig* const configuraciones[] = {&cfg1, &cfg2, &cfg3};
    for (const AxisConfig* cfg : configuraciones) {
        digitalWrite(cfg->pwmPin, LOW);
        pinMode(cfg->pwmPin, OUTPUT);
    }
    if (!m1.begin() || !m2.begin() || !Motor3::begin()) {
        bloquear("Inicializacion de motores fallida. Revisar pines, finales y ajustes.");
        return;
    }
    diagnosticoDisponible = true;
    ayuda();
}

void loop() {
    if (etapa==Etapa::Bloqueado || etapa==Etapa::PosicionamientoIncompleto) {
        Panel::update();
        supervisar();
        const bool reiniciable = etapa==Etapa::Bloqueado || etapa==Etapa::PosicionamientoIncompleto;
        if (diagnosticoDisponible && Panel::blocked()) origenReinicio = OrigenReinicio::Ninguno;
        const bool startFisico = diagnosticoDisponible && Panel::nuevoStart(reiniciable);
        Command orden;
        const bool recibida = comandos.poll(orden);
        if (startFisico && (!recibida || orden.type!=CommandType::Start))
            confirmarReinicio(OrigenReinicio::Pulsador);
        if (recibida) {
            if (orden.type==CommandType::Status || orden.type==CommandType::Encoder) {
                if (diagnosticoDisponible) consultar(orden.type==CommandType::Encoder);
                else printReply(motivo);
            }
            else if (orden.type==CommandType::Help) ayuda();
            else if (reiniciable && orden.type==CommandType::Start) confirmarReinicio(OrigenReinicio::Serie);
            else printReply("Ciclo detenido. Revisar la maquina; dos START o dos I para reiniciar.");
        }
        actualizarIndicador();
        serviceTelemetry();
        return;
    }
    Panel::update();
    const bool stopFisico = Panel::nuevoStop();
    const bool startFisico = Panel::nuevoStart(etapa==Etapa::EsperandoStart || etapa==Etapa::CicloCompletado
                                               || etapa==Etapa::Pausado || startAutorizaHome());
    Command orden;
    const bool recibida = comandos.poll(orden);
    const bool stopPedido = stopFisico || (recibida && orden.type==CommandType::Stop);
    supervisar(stopPedido);
    if (startFisico && !stopPedido && !Panel::blocked()) {
        if (etapa==Etapa::EsperandoStart || etapa==Etapa::CicloCompletado || startAutorizaHome()) iniciar();
        else if (etapa==Etapa::Pausado) {
            if (homeSolicitado) printReply("HOME solicitado; confirmar con I tras la espera.");
            else reanudar();
        }
    }
    if (recibida && orden.type!=CommandType::Stop) {
        switch (orden.type) {
        case CommandType::Start:
        case CommandType::Next:
            if (stopPedido || etapa==Etapa::Bloqueado || etapa==Etapa::PosicionamientoIncompleto || Panel::blocked())
                printReply("Orden bloqueada. Consultar Q.");
            else if (orden.type==CommandType::Start) iniciar();
            else avanzar();
            break;
        case CommandType::Status: consultar(false); break;
        case CommandType::Encoder: consultar(true); break;
        case CommandType::Help: ayuda(); break;
        default: printReply("Comando invalido. Enviar ? para ayuda."); break;
        }
    }

    supervisar();
    if (!stopPedido) continuarCiclo();
    const uint32_t ahoraUs = micros();
    if (etapa!=Etapa::Bloqueado && (m1.busy() || m2.busy())
        && uint32_t(ahoraUs-previoUs)>min(cfg1.maxControlGapUs, cfg2.maxControlGapUs))
        bloquear("Demora excesiva del bucle de control M1/M2.");
    previoUs = ahoraUs;
    for (unsigned i=0; i<2; ++i) {
        if (etapa==Etapa::Bloqueado || etapa==Etapa::PosicionamientoIncompleto) break;
        Axis& eje = *ejes[i];
        const bool esperar = etapa==Etapa::Posicionando && (pausaContacto.activa || ejeEnPausa==static_cast<int>(i));
        if (eje.busy()) eje.update(micros(), millis(), esperar);
        if (eje.hasFault()) bloquearMordaza(i, eje.message());
        if (etapa==Etapa::Sujetando || etapa==Etapa::Posicionando) supervisar();
    }
    if (etapa!=Etapa::Bloqueado && etapa!=Etapa::PosicionamientoIncompleto) Motor3::update();
    supervisar();
    actualizarIndicador();
    serviceTelemetry();
}
