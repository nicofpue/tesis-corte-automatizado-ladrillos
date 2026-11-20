// =====================================================================
// main.cpp - Coordinador del ciclo de corte (Firmware_Cortadora)
// ---------------------------------------------------------------------
// Placa: ESP32-S3-DevKitC-1, framework Arduino (PlatformIO).
// M1 y M2 = mordazas (PID de posicion); M3 = transporte (PWM fijo con rampa);
// sierra = contactor (Panel::on / Panel::off).
//
// setup(): valida pines y parametros, inicializa panel y motores (sin moverlos).
// loop():  lazo unico NO bloqueante (sin delay). En cada vuelta:
//   1) lee panel y comandos serie   2) supervisar(): seguridad y fin de etapa
//   3) continuarCiclo(): pasa a la siguiente etapa cuando la actual esta lista
//   4) actualiza M1, M2 (PID cada 10 ms dentro de Axis::update) y M3
//   5) LED y envio de mensajes.
//
// Ciclo: Inicializando -> Referenciando (HOME) -> EsperandoStart
//   -> Sujetando (cierre hasta CONTACTO) -> Posicionando (centrado)
//   -> Apretando (compresion, si COMPRIMIR_DESPUES_DE_CENTRAR)
//   -> ArrancandoSierra (2 s) -> AvanzandoCorte (M3 a END) -> Retornando (M3 a HOME)
//   -> ApagandoSierra (contactor OFF, espera 20 s) -> Liberando -> CicloCompletado.
// Fuera del ciclo: Pausado (STOP), Emergencia y Bloqueado (fallo).
// =====================================================================
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

// Etapas del ciclo (maquina de estados principal).
// El orden importa: las comparaciones de rango usan Posicionando..ApagandoSierra (pieza sujeta)
// y ArrancandoSierra..ApagandoSierra (fase de sierra).
enum class Etapa {
    Inicializando, Referenciando, EsperandoStart, Sujetando, Posicionando, Apretando,
    ArrancandoSierra, AvanzandoCorte, Retornando, ApagandoSierra,
    Liberando, AbriendoSinPieza, CicloCompletado, Pausado, Emergencia, Bloqueado
};
// Origen del pedido de reinicio en BLOQUEADO (se exigen dos ordenes del mismo origen).
enum class OrigenReinicio { Ninguno, Pulsador, Serie };
// Avance de la compresion de cada mordaza despues del CONTACTO.
enum class Compresion { EsperandoContacto, Confirmando, Avanzando, Completa };

// ---------- Objetos del sistema ----------
const AxisConfig cfg1 = motor1Config(), cfg2 = motor2Config(), cfg3 = motor3Config();
Axis m1(cfg1), m2(cfg2);
Axis* const ejes[] = {&m1, &m2};
const GeometryConfig geometria = geometryConfig();
Commands comandos;

// ---------- Estado del coordinador ----------
// 'lista' = la etapa actual termino y puede pasar a la siguiente.
Etapa etapa = Etapa::Inicializando;
OrigenReinicio origenReinicio = OrigenReinicio::Ninguno;
uint32_t primerReinicioMs = 0;
Etapa etapaPausada = Etapa::Inicializando;
uint32_t pausaDesde = 0;
bool pausaLista = false, homeSolicitado = false;
bool lista = false;
bool homeMordazasIniciado = false;
bool aprieteHecho = false;
bool avisoRetirarPieza = false;
bool diagnosticoDisponible = false;
bool emergenciaDetectada = false;
// Confirmacion de que M3 sigue en HOME mientras arranca la sierra.
struct ConfirmacionHomeArranque {
    bool pendiente = false, activo = false;
    uint32_t desde = 0, cambioDesde = 0;
} homeArranque;
uint32_t previoUs = 0, encendidoDesde = 0, apagadoDesde = 0;
// Centrado: separacion al sujetar, centro final, posicion inicial y objetivos (referencia).
float brechaSujeta = 0, centroCentrado = 0;
float inicioTraslado[2] = {}, referencia[2] = {};
uint32_t inicioPosicionMs = 0;
bool avisoPausaCentrado = false;
int ejeEnPausa = -1;               // Mordaza adelantada en pausa durante el centrado (-1: ninguna).
unsigned pausasSincronismo = 0;    // Veces que se pauso una mordaza adelantada en el centrado actual.
// Tiempo sin CONTACTO de cada mordaza con la pieza sujeta (tolerancia: PERDIDA_CONTACTO_MS).
struct PerdidaContacto {
    bool pendiente = false;
    uint32_t desde = 0;
};
PerdidaContacto perdidasContacto[2];
// Seguimiento de cada mordaza en el cierre: detencion por CONTACTO, confirmacion y compresion.
struct Contacto {
    bool detenido = false, activo = false, confirmado = false;
    uint32_t detenidoDesde = 0, activoDesde = 0;
    Compresion compresion = Compresion::EsperandoContacto;
    float posicionContacto = 0;
    float objetivo = 0;
};
Contacto contactos[2];
// Registro (sin fallo) del movimiento de las mordazas mientras la sierra trabaja.
struct Deriva {
    bool activa = false;
    float x0[2] = {}, maximo[2] = {};
    float maxCentro = 0, maxSeparacion = 0;
} deriva;
const char* motivo = "";
char detalleCentrado[180];

// Nombre de la etapa para el monitor serie.
const char* nombre() {
    switch (etapa) {
    case Etapa::Inicializando: return "INICIALIZANDO";
    case Etapa::Referenciando: return "REFERENCIANDO";
    case Etapa::EsperandoStart: return "ESPERANDO START";
    case Etapa::Sujetando: return "SUJETANDO";
    case Etapa::Posicionando: return "POSICIONANDO";
    case Etapa::Apretando: return "APRETANDO";
    case Etapa::ArrancandoSierra: return "ARRANCANDO SIERRA";
    case Etapa::AvanzandoCorte: return "AVANZANDO CORTE";
    case Etapa::Retornando: return "RETORNANDO";
    case Etapa::ApagandoSierra: return "APAGANDO SIERRA";
    case Etapa::Liberando: return "LIBERANDO";
    case Etapa::AbriendoSinPieza: return "ABRIENDO SIN PIEZA";
    case Etapa::CicloCompletado: return "CICLO COMPLETADO";
    case Etapa::Pausado: return "PAUSADO POR STOP";
    case Etapa::Emergencia: return "EMERGENCIA";
    case Etapa::Bloqueado: return "BLOQUEADO";
    }
    return "?";
}

// true si ya pasaron 'espera' ms desde 'desde' (correcto aunque millis() se desborde).
bool pasoListo(uint32_t ahora, uint32_t desde, uint32_t espera) {
    return uint32_t(ahora - desde) >= espera;
}

// ---------- Registro de tiempos del ciclo ----------
constexpr size_t NUM_ETAPAS = static_cast<size_t>(Etapa::Bloqueado)+1;
struct RegistroCiclo {
    bool activo = false;
    unsigned numero = 0;
    uint32_t inicio = 0, etapaDesde = 0;
    uint32_t ms[NUM_ETAPAS] = {};
} ciclo;

float segundos(Etapa e) { return ciclo.ms[static_cast<size_t>(e)]/1000.0f; }

void comenzarRegistroCiclo() {
    const uint32_t numero = ciclo.numero+1;
    ciclo = {};
    ciclo.activo = true;
    ciclo.numero = numero;
    ciclo.inicio = ciclo.etapaDesde = millis();
}

// Resumen al terminar o interrumpirse un ciclo iniciado con START.
void informarCiclo(bool completo, const char* etapaPrevia) {
    char linea[190];
    const float total = (millis()-ciclo.inicio)/1000.0f;
    if (completo)
        snprintf(linea,sizeof(linea),"CICLO #%u COMPLETO: total=%.1f s | sujecion=%.1f centrado=%.1f apriete=%.1f s",
            ciclo.numero,total,segundos(Etapa::Sujetando),segundos(Etapa::Posicionando),segundos(Etapa::Apretando));
    else
        snprintf(linea,sizeof(linea),"CICLO #%u INTERRUMPIDO en %s tras %.1f s | sujecion=%.1f centrado=%.1f apriete=%.1f s",
            ciclo.numero,etapaPrevia,total,segundos(Etapa::Sujetando),segundos(Etapa::Posicionando),segundos(Etapa::Apretando));
    printReply(linea);
    snprintf(linea,sizeof(linea),"CICLO #%u: arranque=%.1f corte=%.1f retorno=%.1f parada disco=%.1f liberacion=%.1f pausas=%.1f s",
        ciclo.numero,segundos(Etapa::ArrancandoSierra),segundos(Etapa::AvanzandoCorte),segundos(Etapa::Retornando),
        segundos(Etapa::ApagandoSierra),segundos(Etapa::Liberando),segundos(Etapa::Pausado));
    printReply(linea);
    ciclo.activo = false;
}

// Cambio de etapa: suma el tiempo de la anterior, reinicia la supervision que corresponda
// e informa por el monitor. Al terminar o interrumpirse un ciclo imprime su resumen.
void entrar(Etapa siguiente) {
    const uint32_t ahora = millis();
    const char* etapaPrevia = nombre();
    if (ciclo.activo) ciclo.ms[static_cast<size_t>(etapa)] += ahora-ciclo.etapaDesde;
    ciclo.etapaDesde = ahora;
    etapa = siguiente;
    // Fuera de las etapas con pieza sujeta se reinicia la supervision de CONTACTO.
    if (siguiente<=Etapa::Posicionando || siguiente>Etapa::ApagandoSierra) {
        for (auto& perdida : perdidasContacto) perdida = {};
    }
    if (siguiente==Etapa::ArrancandoSierra) homeArranque = {};
    origenReinicio = OrigenReinicio::Ninguno;
    lista = false;
    avisoRetirarPieza = false;
    previoUs = micros();
    printReply(nombre());
    // Fin del ciclo: completo, o interrumpido por fallo, emergencia, HOME pedido con STOP o falta de pieza.
    if (ciclo.activo) {
        if (siguiente==Etapa::CicloCompletado) informarCiclo(true, etapaPrevia);
        else if (siguiente==Etapa::Bloqueado || siguiente==Etapa::Emergencia
              || siguiente==Etapa::Referenciando || siguiente==Etapa::EsperandoStart)
            informarCiclo(false, etapaPrevia);
    }
}

// Marca la etapa actual como terminada (una sola vez) e imprime el mensaje.
void completar(const char* mensaje) {
    if (lista) return;
    lista = true;
    printReply(mensaje);
}

// Apaga todo: contactor OFF y PWM = 0 en los tres motores.
void detenerSalidas() {
    Panel::off();
    m1.stop();
    m2.stop();
    Motor3::stop();
}

// ---------- Registro de deriva durante el corte ----------
void iniciarDeriva() {
    deriva = {};
    deriva.activa = true;
    deriva.x0[0] = m1.position();
    deriva.x0[1] = m2.position();
}

void actualizarDeriva() {
    if (!deriva.activa) return;
    const float d1 = m1.position()-deriva.x0[0], d2 = m2.position()-deriva.x0[1];
    deriva.maximo[0] = fmaxf(deriva.maximo[0], fabsf(d1));
    deriva.maximo[1] = fmaxf(deriva.maximo[1], fabsf(d2));
    // Centro estimado: (aR-aL+x1-x2)/2. Separacion: aL+aR-x1-x2.
    deriva.maxCentro = fmaxf(deriva.maxCentro, fabsf((d1-d2)*0.5f));
    deriva.maxSeparacion = fmaxf(deriva.maxSeparacion, fabsf(d1+d2));
}

void reportarDeriva() {
    if (!deriva.activa) return;
    char linea[180];
    snprintf(linea, sizeof(linea),
        "DERIVA durante la sierra (registro): M1 max=%.3f M2 max=%.3f; centro max=%.3f; separacion max=%.3f mm",
        deriva.maximo[0], deriva.maximo[1], deriva.maxCentro, deriva.maxSeparacion);
    printReply(linea);
    deriva.activa = false;
}

// FALLO: detiene todo, pasa a BLOQUEADO y guarda la causa (se consulta con Q).
// Solo se sale reiniciando: dos START o dos I en menos de 5 s.
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
    reportarDeriva();
}

// Igual que bloquear(), indicando que mordaza fallo.
void bloquearMordaza(unsigned i, const char* causa) {
    snprintf(detalleCentrado, sizeof(detalleCentrado), "M%u: %s", i+1, causa);
    bloquear(detalleCentrado);
}

// Reinicio desde BLOQUEADO: segunda orden del mismo origen en menos de 5 s -> reinicia la ESP32.
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

// M3 referenciado, en HOME, sin END y quieto: condicion para mover las mordazas.
bool homeM3() {
    return Motor3::isHomed() && Motor3::homePressed() && !Motor3::endPressed() && !Motor3::busy();
}

// Mientras arranca la sierra, M3 debe seguir en HOME; se tolera un rebote breve de la senal.
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

// Ambas mordazas referenciadas, en HOME y quietas.
bool mordazasEnHome() {
    return m1.isHomed() && m2.isHomed() && m1.homePressed() && m2.homePressed()
        && !m1.endPressed() && !m2.endPressed() && !m1.busy() && !m2.busy();
}

// Estado de los sensores de CONTACTO.
bool algunContacto() { return m1.contactPressed() || m2.contactPressed(); }
bool ambosContactos() { return m1.contactPressed() && m2.contactPressed(); }

// Contactor OFF y registro del instante, para contar la espera de parada del disco.
void apagar() {
    Panel::off();
    apagadoDesde = millis();
}

// STOP: la primera orden pausa con todo apagado; la segunda (ya en pausa) pide volver a HOME.
void parar() {
    if (etapa==Etapa::Emergencia) return;
    if (etapa==Etapa::Pausado) {
        homeSolicitado = true;
        printReply("HOME solicitado. Esperar la parada (25 s) y confirmar con START o I.");
        return;
    }
    etapaPausada = etapa;
    pausaLista = lista;
    homeSolicitado = false;
    detenerSalidas();
    pausaDesde = millis();
    entrar(Etapa::Pausado);
    printReply("STOP: salidas OFF. START o I reanuda; otro STOP solicita HOME.");
}

// Al arrancar: ningun pin repetido ni canal PWM compartido entre motores.
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

// Referenciacion: primero M3 a HOME; las mordazas se referencian despues (en supervisar).
void comenzarHome() {
    Panel::off();
    homeMordazasIniciado = false;
    entrar(Etapa::Referenciando);
    if (!Motor3::seekLimit(-1)) bloquear("No se pudo iniciar HOME de M3.");
}

// Inicia el traslado de centrado: ambas mordazas hacia sus objetivos con la pieza sujeta.
void moverAReferencias() {
    inicioPosicionMs = millis();
    avisoPausaCentrado = false;
    ejeEnPausa = -1;
    pausasSincronismo = 0;
    if (!m1.moveHeldTo(referencia[0])) { bloquearMordaza(0, m1.message()); return; }
    if (!m2.moveHeldTo(referencia[1])) bloquearMordaza(1, m2.message());
}

// Abre las mordazas (HOME). Solo con M3 en HOME, para no soltar la pieza frente al disco.
void abrirMordazas(Etapa siguiente) {
    if (!homeM3()) { bloquear("M3 fuera de HOME al abrir las mordazas."); return; }
    detenerSalidas();
    reportarDeriva();
    entrar(siguiente);
    if (!m1.home() || !m2.home()) bloquear("No se pudo iniciar HOME de las mordazas.");
}

// Aceptacion del centrado: cada mordaza a +-TOLERANCIA_CENTRADO_MM (2 mm) de su objetivo.
bool posicionValida(float x1, float x2) {
    return fabsf(x1-referencia[0])<=TOLERANCIA_CENTRADO_MM
        && fabsf(x2-referencia[1])<=TOLERANCIA_CENTRADO_MM;
}

// Reanudar tras STOP segun la etapa pausada. El corte y el retorno no se reanudan: se pide HOME.
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
    case Etapa::Apretando:
        if (!homeM3()) { bloquear("M3 fuera de HOME al reanudar la sujecion."); return; }
        entrar(etapaPausada);
        for (unsigned i=0; i<2; ++i) {
            Axis& eje = *ejes[i];
            Contacto& contacto = contactos[i];
            if (contacto.detenido && !eje.contactPressed()) {
                bloquear("CONTACTO perdido durante la pausa."); return;
            }
            if (contacto.detenido) {
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
                bloquearMordaza(i, eje.message()); return;
            }
        }
        break;
    case Etapa::Posicionando:
    case Etapa::ArrancandoSierra:
        if (!homeM3() || !ambosContactos()) {
            bloquear("Revisar HOME de M3 y ambos CONTACTOS."); return;
        }
        if (etapaPausada==Etapa::ArrancandoSierra && !pasoListo(millis(), pausaDesde, PARADA_MS)) {
            printReply("Esperar parada de la sierra antes de reanudar."); return;
        }
        entrar(Etapa::Posicionando);
        if ((pausaLista || etapaPausada==Etapa::ArrancandoSierra) && posicionValida(m1.position(),m2.position())) {
            completar("LISTO: centrado confirmado para reanudar el ciclo.");
        } else {
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

// Espera antes de autorizar HOME: 20 s tras EMERGENCIA; 25 s al encender o tras STOP.
uint32_t esperaRecuperacionHome() {
    return etapa==Etapa::Emergencia ? ESPERA_EMERGENCIA_MS : PARADA_MS;
}

// START habilitado para autorizar HOME (al encender o tras EMERGENCIA, cumplida la espera).
bool startAutorizaHome() {
    return lista && (etapa==Etapa::Inicializando || etapa==Etapa::Emergencia);
}

// START fisico o I: autoriza HOME, inicia el ciclo, reanuda una pausa o confirma el HOME pedido con STOP.
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
        aprieteHecho = false;
        deriva = {};
        for (auto& contacto : contactos) contacto = {};
        comenzarRegistroCiclo();
        entrar(Etapa::Sujetando);
        if (!m1.approachTo(cfg1.maxTravelMm) || !m2.approachTo(cfg2.maxTravelMm))
            bloquear("No se pudo iniciar cierre de las mordazas.");
    } else {
        printReply("START no aplica en esta etapa. Consultar Q.");
    }
}

// Imprime ancho y centro estimados de la pieza (lineas SUJETA, CENTRADO y APRETADA).
void informarApertura(const char* estado, float x1, float x2) {
    const auto pieza = estimatedGrip(geometria, x1, x2);
    char linea[160];
    snprintf(linea,sizeof(linea),"%s: apertura EST=%.3f mm; separacion REF=%.3f mm; centro EST=%+.3f mm (+derecha)",
        estado,pieza.width,faceGap(geometria,x1,x2),pieza.center);
    printReply(linea);
}

// CENTRADO: calcula el centro estimado de la pieza y el traslado que lo lleva al centro fijo,
// mas el desfase DESFASE_CENTRO_MM. M1 y M2 se mueven la misma distancia en el mismo
// sentido, asi la separacion (y la presion sobre la pieza) no cambia.
void iniciarPosicionamiento() {
    if (!ambosContactos()) {
        bloquear("Se requieren ambos CONTACTOS para centrar."); return;
    }
    inicioTraslado[0] = m1.position();
    inicioTraslado[1] = m2.position();
    const auto pieza = estimatedGrip(geometria, inicioTraslado[0], inicioTraslado[1]);
    if (!isfinite(pieza.center) || !isfinite(pieza.width) || pieza.width<=0) {
        bloquear("Centrado rechazado: geometria invalida."); return;
    }
    const float traslado = fabsf(pieza.center)<=geometria.centerToleranceMm ? 0.0f : -pieza.center;
    // El desfase desplaza ambas mordazas hacia la derecha la misma distancia.
    const float movimiento = traslado + DESFASE_CENTRO_MM;
    const float objetivoM1 = inicioTraslado[0]+movimiento;
    const float objetivoM2 = inicioTraslado[1]-movimiento;
    ShiftRange recorrido;
    if (!translationRange(inicioTraslado[0], inicioTraslado[1], cfg1.maxTravelMm, cfg2.maxTravelMm, recorrido)
        || objetivoM1<0 || objetivoM1>cfg1.maxTravelMm || objetivoM2<0 || objetivoM2>cfg2.maxTravelMm
        || (objetivoM1>inicioTraslado[0] && m1.endPressed())
        || (objetivoM1<inicioTraslado[0] && m1.homePressed())
        || (objetivoM2>inicioTraslado[1] && m2.endPressed())
        || (objetivoM2<inicioTraslado[1] && m2.homePressed())) {
        snprintf(detalleCentrado,sizeof(detalleCentrado),
            "Centrado rechazado: movimiento de %+.3f mm fuera de recorrido o bloqueado por un final.",movimiento);
        bloquear(detalleCentrado);return;
    }
    referencia[0] = objetivoM1;
    referencia[1] = objetivoM2;
    brechaSujeta = faceGap(geometria, inicioTraslado[0], inicioTraslado[1]);
    entrar(Etapa::Posicionando);
    char linea[180];
    snprintf(linea,sizeof(linea),"CENTRADO EST: traslado=%+.3f mm + desfase=%+.3f mm (+derecha); objetivos M1=%.3f M2=%.3f; separacion REF=%.3f mm",
        traslado,DESFASE_CENTRO_MM,referencia[0],referencia[1],brechaSujeta);
    printReply(linea);
    if (movimiento==0.0f) {
        centroCentrado = pieza.center;
        informarApertura("CENTRADO",inicioTraslado[0],inicioTraslado[1]);
        completar("LISTO: centro estimado dentro de tolerancia.");
        return;
    }
    moverAReferencias();
}

// Sierra ON (contactor) con ambos CONTACTOS activos; inicia el registro de deriva.
void arrancarSierra() {
    if (!ambosContactos()) {
        bloquear("Se requieren ambos CONTACTOS para accionar el contactor."); return;
    }
    entrar(Etapa::ArrancandoSierra);
    iniciarDeriva();
    Panel::on();
    encendidoDesde = millis();
    char linea[140];
    snprintf(linea,sizeof(linea),"M3: PWM objetivo %.1f%%; rampa %lu ms; proteccion sin cambio de encoder %lu ms",
        cfg3.endSeekPwm,static_cast<unsigned long>(M3_SEEK_RAMP_MS),
        static_cast<unsigned long>(cfg3.noCountsTimeoutMs));
    printReply(linea);
}

// Compresion final con la pieza ya centrada (solo si COMPRIMIR_DESPUES_DE_CENTRAR).
void iniciarApriete() {
    if (!ambosContactos()) {
        bloquear("Se requieren ambos CONTACTOS para apretar la pieza."); return;
    }
    entrar(Etapa::Apretando);
    const uint32_t ahora = millis();
    for (unsigned i=0; i<2; ++i) {
        Axis& eje = *ejes[i];
        Contacto& contacto = contactos[i];
        contacto = {};
        contacto.detenido = true;
        contacto.detenidoDesde = ahora;
        contacto.posicionContacto = eje.position();
        contacto.objetivo = fminf(contacto.posicionContacto+COMPRESION_MM, eje.config().maxTravelMm);
        contacto.compresion = Compresion::Confirmando;
    }
}

// Paso a la siguiente etapa cuando la actual quedo 'lista'. Aqui se define el orden del ciclo.
void avanzar() {
    if (!lista) return;
    switch (etapa) {
    case Etapa::Referenciando:
        if (!mordazasEnHome() || !homeM3()) {
            bloquear("Revisar los tres HOME."); return;
        }
        if (algunContacto()) {
            if (!avisoRetirarPieza) {
                printReply("Retirar la pieza: hay CONTACTO activo con las mordazas en HOME.");
                avisoRetirarPieza = true;
            }
            return;
        }
        entrar(Etapa::EsperandoStart);
        printReply("Preparar pieza y pulsar START o enviar I.");
        break;
    case Etapa::Sujetando:
        iniciarPosicionamiento();
        break;
    case Etapa::Posicionando:
        if (COMPRIMIR_DESPUES_DE_CENTRAR && !aprieteHecho) iniciarApriete();
        else arrancarSierra();
        break;
    case Etapa::Apretando:
        arrancarSierra();
        break;
    case Etapa::ArrancandoSierra:
        if (homeArranque.pendiente || !homeM3()) return;
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
        completar("Retirar pieza. Una nueva orden START inicia otro ciclo.");
        break;
    default:
        break;
    }
}

// Todas las transiciones del ciclo son automaticas al quedar LISTO.
void continuarCiclo() {
    if (!lista || Panel::blocked()) return;
    switch (etapa) {
    case Etapa::Referenciando:
    case Etapa::Sujetando:
    case Etapa::Posicionando:
    case Etapa::Apretando:
    case Etapa::ArrancandoSierra:
    case Etapa::AvanzandoCorte:
    case Etapa::ApagandoSierra:
    case Etapa::Liberando:
        avanzar();
        break;
    default:
        break;
    }
}

// Las mordazas llegaron al final sin tocar la pieza: sin contacto vuelven a HOME (sin pieza);
// con contacto parcial es un fallo.
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
        if (COMPRIMIR_DESPUES_DE_CENTRAR && etapa==Etapa::Sujetando) {
            // Primero se centra con la presion del contacto; la compresion va en APRETANDO.
            contacto.compresion = Compresion::Completa;
            return;
        }
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

// Confirmacion de CONTACTO (30 ms continuos) comun a SUJETANDO y APRETANDO.
bool actualizarContacto(unsigned i, uint32_t ahora) {
    Contacto& contacto = contactos[i];
    const bool activo = ejes[i]->contactPressed();
    if (activo && !contacto.activo) contacto.activoDesde = ahora;
    contacto.activo = activo;
    contacto.confirmado = activo && pasoListo(ahora, contacto.activoDesde, ejes[i]->config().limitConfirmMs);
    return contacto.confirmado;
}

// Ambas mordazas detenidas al menos contactSettlingMs (300 ms) antes de medir la pieza.
bool estabilizados(uint32_t ahora) {
    return pasoListo(ahora, contactos[0].detenidoDesde, geometria.contactSettlingMs)
        && pasoListo(ahora, contactos[1].detenidoDesde, geometria.contactSettlingMs);
}

// SUJETANDO: cada mordaza cierra a PWM fijo hasta su CONTACTO, se detiene y, segun el
// orden configurado, comprime. Si llega al final sin contacto se trata como 'sin pieza'.
void atenderCierre(uint32_t ahora) {
    bool ambos = true;
    for (unsigned i=0; i<2; ++i) {
        Axis& eje = *ejes[i];
        Contacto& contacto = contactos[i];
        if (eje.endPressed() || eje.state()==AxisState::EndLimit) { cierreSinSujecion(); return; }
        if (!contacto.detenido && (eje.contactPressed() || eje.state()==AxisState::ContactLimit)) {
            eje.haltContact();
            contacto.detenido = true;
            contacto.detenidoDesde = ahora;
            contacto.posicionContacto = eje.position();
            // Acortar el avance cuando el contacto queda cerca del final del recorrido.
            contacto.objetivo = fminf(contacto.posicionContacto+COMPRESION_MM,
                                      eje.config().maxTravelMm);
            contacto.compresion = Compresion::Confirmando;
            printReply(i==0 ? "M1 detenido por CONTACTO." : "M2 detenido por CONTACTO.");
        }
        if (!contacto.detenido && !eje.busy()) {
            if (eje.state()==AxisState::Reached) cierreSinSujecion();
            else bloquear("Una mordaza se detuvo antes de confirmar CONTACTO.");
            return;
        }
        const bool confirmado = actualizarContacto(i, ahora);
        atenderCompresion(i, ahora);
        if (etapa==Etapa::Bloqueado) return;
        ambos &= contacto.compresion==Compresion::Completa && confirmado;
    }
    if (ambos && !lista && estabilizados(ahora)) {
        informarApertura("SUJETA",m1.position(),m2.position());
        completar(COMPRIMIR_DESPUES_DE_CENTRAR ? "LISTO: contactos confirmados. Centrando con presion ligera."
                                               : "LISTO: contactos confirmados y estabilizados. Centrando a continuacion.");
    }
}

// APRETANDO: compresion final con la pieza centrada; esa posicion queda como referencia.
void atenderApriete(uint32_t ahora) {
    if (lista) return;
    bool ambos = true;
    for (unsigned i=0; i<2; ++i) {
        const bool confirmado = actualizarContacto(i, ahora);
        atenderCompresion(i, ahora);
        if (etapa==Etapa::Bloqueado) return;
        ambos &= contactos[i].compresion==Compresion::Completa && confirmado;
    }
    if (!ambos || !estabilizados(ahora)) return;
    const float x1=m1.position(), x2=m2.position();
    informarApertura("APRETADA",x1,x2);
    char linea[140];
    snprintf(linea,sizeof(linea),"APRIETE: cambio del centro EST frente al centrado=%+.3f mm (registro).",
        estimatedGrip(geometria,x1,x2).center-centroCentrado);
    printReply(linea);
    // La pieza queda sujeta en esta posicion: es la nueva referencia para reanudar tras STOP.
    referencia[0] = x1;
    referencia[1] = x2;
    aprieteHecho = true;
    completar("LISTO: pieza apretada.");
}

// POSICIONANDO: supervisa el traslado de centrado. Cuando ambas mordazas se detienen evalua
// la tolerancia (+-2 mm por mordaza y +-5 mm de separacion). Mientras se mueven, las
// sincroniza: pausa a la adelantada y da un pequeno empuje a la rezagada.
void atenderPosicionamiento(uint32_t ahora) {
    if (lista) return;
    if (pasoListo(ahora, inicioPosicionMs, PLAZO_CENTRADO_MS)) {
        snprintf(detalleCentrado,sizeof(detalleCentrado),
            "Centrado sin terminar en %lu ms: M1 x=%.3f obj=%.3f; M2 x=%.3f obj=%.3f; CONTACTO M1=%d M2=%d.",
            static_cast<unsigned long>(PLAZO_CENTRADO_MS), m1.position(), referencia[0],
            m2.position(), referencia[1], m1.contactPressed(), m2.contactPressed());
        bloquear(detalleCentrado);
        return;
    }
    for (const Axis* eje : ejes) {
        if (!eje->busy() && eje->state()!=AxisState::Reached) {
            bloquear("Un sensor impidio llegar a la referencia de mordaza."); return;
        }
    }
    if (!ambosContactos() && !avisoPausaCentrado) {
        char aviso[120];
        snprintf(aviso,sizeof(aviso),"Centrado en pausa: falta CONTACTO (M1=%d M2=%d); continua al recuperarlo.",
            m1.contactPressed(), m2.contactPressed());
        printReply(aviso);
        avisoPausaCentrado = true;
    }
    if (!m1.busy() && !m2.busy()) {
        ejeEnPausa = -1;
        const float x1=m1.position(), x2=m2.position();
        for (size_t i=0; i<2; ++i) {
            if (ejes[i]->stalledHeld()) {
                char aviso[140];
                snprintf(aviso,sizeof(aviso),"M%u se detuvo sin avance (con CONTACTO) durante el centrado; error=%+.3f mm (lim %.3f).",
                    static_cast<unsigned>(i+1), referencia[i]-ejes[i]->position(), TOLERANCIA_CENTRADO_MM);
                printReply(aviso);
            }
        }
        const float separacion=faceGap(geometria,x1,x2);
        // Comparar contra la separacion que dejan los objetivos.
        const float brechaObjetivo = faceGap(geometria, referencia[0], referencia[1]);
        const float cambio = separacion-brechaObjetivo;
        if (!posicionValida(x1,x2) || fabsf(cambio)>TOLERANCIA_SEPARACION_MM) {
            snprintf(detalleCentrado,sizeof(detalleCentrado),"Centrado fuera de tolerancia: error M1=%+.3f M2=%+.3f (lim %.3f); error separacion=%+.3f (lim %.3f) mm.",
                referencia[0]-x1,referencia[1]-x2,TOLERANCIA_CENTRADO_MM,
                cambio,TOLERANCIA_SEPARACION_MM);
            bloquear(detalleCentrado);
            return;
        }
        centroCentrado = estimatedGrip(geometria,x1,x2).center;
        char linea[140];
        snprintf(linea,sizeof(linea),"CENTRADO: error final M1=%+.3f M2=%+.3f mm; tiempo=%lu ms; pausas de sincronismo=%u.",
            referencia[0]-x1, referencia[1]-x2, static_cast<unsigned long>(ahora-inicioPosicionMs), pausasSincronismo);
        printReply(linea);
        informarApertura("CENTRADO",x1,x2);
        completar("LISTO: centrado OK.");
        return;
    }
    // Acoplamiento cruzado: ayuda al eje rezagado para conservar la separacion.
    const float deltaM1 = referencia[0]-inicioTraslado[0];
    const float deltaM2 = referencia[1]-inicioTraslado[1];
    if (deltaM1*deltaM2>=0) return; // El ajuste de sincronismo requiere sentidos opuestos.
    const int8_t sentido = deltaM1>0 ? 1 : -1;
    const float avanceM2 = m2.position()-inicioTraslado[1];
    const float fraccion = constrain(avanceM2/deltaM2, 0.0f, 1.0f);
    const float error = (m1.position()-inicioTraslado[0]+avanceM2-(deltaM1+deltaM2)*fraccion)*sentido;
    // Pausa de la mordaza adelantada para conservar la separacion (como PruebaSujecionDosMotores).
    // Histeresis: pausa por encima de heldSyncPauseMm y reanuda por debajo de heldSyncResumeMm.
    const int adelantado = error>0 ? 0 : 1;
    const int pausaPrevia = ejeEnPausa;
    const float umbral = pausaPrevia==adelantado ? geometria.heldSyncResumeMm : geometria.heldSyncPauseMm;
    ejeEnPausa = -1;
    const Axis& lider = *ejes[adelantado];
    const int8_t sentidoLider = adelantado==0 ? sentido : -sentido;
    // No pausar si el adelantado ya esta llegando: debe poder asentarse o corregir.
    if (fabsf(error)>umbral && lider.busy() && ejes[1-adelantado]->busy()
        && (referencia[adelantado]-lider.position())*sentidoLider>TOLERANCIA_PARADA_CENTRADO_MM) {
        ejeEnPausa = adelantado;
        if (pausaPrevia!=adelantado) ++pausasSincronismo;
    }
    const float ajuste = constrain(geometria.heldSyncGainPwmPerMm*error,
                                   -geometria.maxHeldSyncPwm, geometria.maxHeldSyncPwm);
    m1.synchronization(-ajuste, sentido);
    m2.synchronization(ajuste, -sentido);
}

// Con la pieza sujeta (centrado incluido) se admite una perdida breve de CONTACTO por vibracion.
bool confirmarContactosSujetos(uint32_t ahora) {
    const bool activo[] = {m1.contactPressed(), m2.contactPressed()};
    uint32_t inactivoMs[2] = {};
    for (unsigned i=0; i<2; ++i) {
        auto& perdida = perdidasContacto[i];
        if (activo[i]) perdida = {};
        else {
            if (!perdida.pendiente) perdida = {true, ahora};
            inactivoMs[i] = ahora-perdida.desde;
        }
    }
    if (inactivoMs[0]<PERDIDA_CONTACTO_MS && inactivoMs[1]<PERDIDA_CONTACTO_MS) return true;
    snprintf(detalleCentrado,sizeof(detalleCentrado),
        "Perdida de CONTACTO en %s: M1=%d (%lu ms inactivo), M2=%d (%lu ms inactivo); limite=%lu ms.",
        nombre(), activo[0], static_cast<unsigned long>(inactivoMs[0]), activo[1],
        static_cast<unsigned long>(inactivoMs[1]), static_cast<unsigned long>(PERDIDA_CONTACTO_MS));
    bloquear(detalleCentrado);
    return false;
}

// Supervision en cada vuelta: EMERGENCIA, STOP, fallos de ejes, sensores en conflicto,
// CONTACTO con la pieza sujeta y la condicion de fin de cada etapa.
void supervisar(bool stopPedido=false) {
    if (diagnosticoDisponible && Panel::emergencyActive()) {
        if (!emergenciaDetectada) {
            detenerSalidas();
            reportarDeriva();
            apagadoDesde = millis();
            entrar(Etapa::Emergencia);
            printReply("Cuenta de parada iniciada al detectar EMERGENCIA. Restablecer el circuito antes de autorizar HOME.");
        }
        emergenciaDetectada = true;
        lista = false;
        return;
    }
    emergenciaDetectada = false;
    if (etapa==Etapa::Bloqueado) return;
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
    if (sujeta && !confirmarContactosSujetos(ahora)) return;
    if (etapa>=Etapa::ArrancandoSierra && etapa<=Etapa::ApagandoSierra) actualizarDeriva();

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
    case Etapa::Apretando:
        if (!homeM3()) bloquear("M3 salio de HOME durante el apriete.");
        else atenderApriete(ahora);
        break;
    case Etapa::ArrancandoSierra:
        if (!confirmarHomeDuranteArranque(ahora)) break;
        if (pasoListo(ahora, encendidoDesde, ARRANQUE_SIERRA_MS))
            completar("LISTO: tiempo de arranque cumplido.");
        break;
    case Etapa::AvanzandoCorte:
        if (Motor3::busy()) break;
        if (!Motor3::endPressed() || Motor3::homePressed()) bloquear("M3 no confirma END.");
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
            completar("LISTO: tiempo de parada cumplido. Apertura automatica.");
        break;
    case Etapa::Liberando:
    case Etapa::AbriendoSinPieza:
        if (!homeM3()) bloquear("M3 salio de HOME durante la liberacion.");
        else if (!m1.busy() && !m2.busy()) {
            if (!mordazasEnHome() || algunContacto()) bloquear("Liberacion: revisar HOME y CONTACTO.");
            else if (etapa==Etapa::AbriendoSinPieza) {
                entrar(Etapa::EsperandoStart);
                printReply("LISTO: mordazas en HOME. Colocar pieza y pulsar START o enviar I.");
            } else completar("LISTO: mordazas abiertas.");
        }
        break;
    default:
        break;
    }
}

// Comandos Q (estado) y E (diagnostico con pines y encoders).
void consultar(bool diagnostico) {
    char linea[180];
    snprintf(linea, sizeof(linea), "%s | %s | orden: %s | STOP/EMERG=%d",
             nombre(), lista ? "LISTO" : "EN ESPERA",
             COMPRIMIR_DESPUES_DE_CENTRAR ? "contacto-centrado-apriete" : "contacto-compresion-centrado",
             Panel::blocked());
    printReply(linea);
    if (etapa==Etapa::Bloqueado) printReply(motivo);
    if (etapa==Etapa::Pausado) {
        printReply(homeSolicitado ? "HOME solicitado: confirmar con START o I tras la espera."
                                 : "Pausa: START o I reanuda; otra orden STOP solicita HOME.");
        if (homeSolicitado || etapaPausada==Etapa::ArrancandoSierra) {
            const uint32_t transcurrido = millis()-pausaDesde;
            snprintf(linea,sizeof(linea),"Espera de parada restante: %lu ms",
                static_cast<unsigned long>(transcurrido>=PARADA_MS ? 0 : PARADA_MS-transcurrido));
            printReply(linea);
        }
    }
    if (etapa==Etapa::ApagandoSierra || etapa==Etapa::Inicializando || etapa==Etapa::Emergencia) {
        const uint32_t transcurrido = millis()-apagadoDesde;
        const uint32_t espera = etapa==Etapa::ApagandoSierra ? APERTURA_TRAS_PARADA_MS : esperaRecuperacionHome();
        snprintf(linea, sizeof(linea), "Espera de parada restante: %lu ms",
                 static_cast<unsigned long>(transcurrido>=espera ? 0 : espera-transcurrido));
        printReply(linea);
    }
    if (deriva.activa) {
        snprintf(linea, sizeof(linea), "Deriva hasta ahora: M1 max=%.3f M2 max=%.3f; centro max=%.3f; separacion max=%.3f mm",
            deriva.maximo[0], deriva.maximo[1], deriva.maxCentro, deriva.maxSeparacion);
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

// Comando ? (ayuda).
void ayuda() {
    printReply("Cortadora de ladrillos: ciclo automatico con contactor real de la sierra.");
    printReply("I = igual que START: HOME al encender o recuperar, inicia ciclo, reanuda o confirma HOME tras STOP.");
    printReply("S o X = STOP (primero pausa, segundo pide HOME) | Q = estado | E = diagnostico | ? = ayuda");
    printReply(PanelConfig::physicalStopEnabled ? "STOP fisico habilitado."
        : "STOP fisico deshabilitado. Usar S/X para pausar o solicitar HOME.");
    printReply("LED: verde fijo=pulse START para cortar; verde titila=pulse START (HOME o reanudar);");
    printReply("     amarillo fijo=trabajando; amarillo titila=espere; azul=retire la pieza;");
    printReply("     rojo fijo=emergencia; rojo titila=fallo (dos START para reiniciar).");
}

// LED de la placa: cuatro colores, fijo o titilando.
void actualizarIndicador() {
    using Panel::Color;
    const uint32_t ahora = millis();
    switch (etapa) {
    case Etapa::Bloqueado:
        Panel::indicar(Color::Rojo, true); break;
    case Etapa::Inicializando:
    case Etapa::Emergencia:
        Panel::indicar(lista ? Color::Verde : Color::Amarillo, true); break;
    case Etapa::Pausado: {
        const bool esperando = (homeSolicitado || etapaPausada==Etapa::ArrancandoSierra)
            && !pasoListo(ahora, pausaDesde, PARADA_MS);
        Panel::indicar(esperando ? Color::Amarillo : Color::Verde, true); break;
    }
    case Etapa::EsperandoStart:
        Panel::indicar(Color::Verde, false); break;
    case Etapa::CicloCompletado:
        Panel::indicar(Color::Azul, false); break;
    case Etapa::Referenciando:
        // HOME terminado con pieza entre las mordazas: retirar la pieza.
        if (lista && algunContacto()) Panel::indicar(Color::Azul, false);
        else Panel::indicar(Color::Amarillo, false);
        break;
    case Etapa::ApagandoSierra:
        Panel::indicar(Color::Amarillo, true); break;
    default:
        Panel::indicar(Color::Amarillo, false); break;
    }
}
}

// ========================= setup(): una vez al encender =========================
// Valida pines y parametros, deja el contactor OFF y los PWM en 0 e inicializa los ejes.
// No mueve ningun motor: el HOME requiere START o I despues de la espera inicial.
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
        || !isfinite(geometria.centerToleranceMm) || geometria.centerToleranceMm<0
        || !isfinite(geometria.heldSyncGainPwmPerMm) || geometria.heldSyncGainPwmPerMm<0
        || !isfinite(geometria.maxHeldSyncPwm) || geometria.maxHeldSyncPwm<0
        || !isfinite(DESFASE_CENTRO_MM) || PLAZO_CENTRADO_MS>min(cfg1.moveTimeoutMs,cfg2.moveTimeoutMs)) {
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

// ===================== loop(): lazo principal no bloqueante ======================
// En BLOQUEADO solo atiende consultas y el reinicio. En operacion normal:
// panel -> comandos -> supervisar -> continuar ciclo -> ejes -> LED -> telemetria.
void loop() {
    if (etapa==Etapa::Bloqueado) {
        Panel::update();
        supervisar();
        const bool reiniciable = etapa==Etapa::Bloqueado;
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
    // START fisico y I hacen lo mismo, incluida la confirmacion del HOME pedido con doble STOP.
    if (startFisico && !stopPedido && !Panel::blocked()) iniciar();
    if (recibida && orden.type!=CommandType::Stop) {
        switch (orden.type) {
        case CommandType::Start:
            if (stopPedido || etapa==Etapa::Bloqueado || Panel::blocked())
                printReply("Orden bloqueada. Consultar Q.");
            else iniciar();
            break;
        case CommandType::Status: consultar(false); break;
        case CommandType::Encoder: consultar(true); break;
        case CommandType::Help: ayuda(); break;
        default: printReply("Comando invalido. Enviar ? para ayuda."); break;
        }
    }

    supervisar();
    if (!stopPedido) continuarCiclo();
    // Proteccion: si una vuelta tarda mas de 100 ms con mordazas en movimiento, se bloquea.
    const uint32_t ahoraUs = micros();
    if (etapa!=Etapa::Bloqueado && (m1.busy() || m2.busy())
        && uint32_t(ahoraUs-previoUs)>min(cfg1.maxControlGapUs, cfg2.maxControlGapUs))
        bloquear("Demora excesiva del bucle de control M1/M2.");
    previoUs = ahoraUs;
    // Centrado: si falta un CONTACTO, ambas mordazas quedan con PWM=0 hasta recuperarlo.
    const bool pausaCentrado = etapa==Etapa::Posicionando && !lista && !ambosContactos();
    for (unsigned i=0; i<2; ++i) {
        if (etapa==Etapa::Bloqueado) break;
        Axis& eje = *ejes[i];
        const bool esperar = pausaCentrado
            || (etapa==Etapa::Posicionando && !lista && ejeEnPausa==static_cast<int>(i));
        if (eje.busy()) eje.update(micros(), millis(), esperar);
        if (eje.hasFault()) bloquearMordaza(i, eje.message());
        if (etapa==Etapa::Sujetando || etapa==Etapa::Posicionando || etapa==Etapa::Apretando) supervisar();
    }
    // Transporte (M3).
    if (etapa!=Etapa::Bloqueado) Motor3::update();
    supervisar();
    actualizarIndicador();
    serviceTelemetry();
}
