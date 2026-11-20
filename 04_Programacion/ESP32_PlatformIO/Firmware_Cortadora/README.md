# Firmware de la cortadora automatizada de ladrillos

Proyecto PlatformIO para la ESP32-S3-DevKitC-1 (C++, framework Arduino). Coordina las dos mordazas (M1 y M2), el eje de avance y retorno (M3) y el contactor de la sierra.

Es un **único firmware, sin perfiles**, basado en la secuencia de `PruebaSecuenciaManual` (la que se usó para los cortes), con las correcciones descritas en "Cambios respecto a PruebaSecuenciaManual". `PruebaSecuenciaManual` y `PruebaSujecionDosMotores` se conservan sin cambios como códigos de prueba.

## Compilar y cargar

```powershell
pio run -e cortadora -t upload
pio device monitor
```

El monitor serie funciona a 115200 baudios por el USB nativo (`ARDUINO_USB_CDC_ON_BOOT=1`).

Cada sesión de `pio device monitor` se guarda en `logs/device-monitor-AAMMDD-HHMMSS.log`, con la hora de cada línea (filtros `time` y `log2file` en `platformio.ini`). Al terminar o interrumpirse un ciclo, el firmware imprime dos líneas `CICLO #n` con el tiempo total y el de cada etapa.

## Funciones de seguridad

| Función | Estado |
|---|---|
| Parada de emergencia (GPIO43, NC) | Leída. El circuito externo corta la energía de los motores y el contactor; la ESP32 sigue encendida, anula salidas y entra en EMERGENCIA. |
| STOP físico (GPIO21) | **Deshabilitado** (`physicalStopEnabled=false`) por falsos disparos. Se usa `S` o `X` en el monitor. Si se habilita sin conectarlo, el firmware lo lee como STOP pulsado permanentemente. |
| Sensor de cubierta (GPIO47) | No instalado; no se lee. |
| Giro de la sierra | Sin sensor. Arranque y parada se gestionan con tiempos (2 s y 20 s). |

## Ciclo automático

`COMPRIMIR_DESPUES_DE_CENTRAR` (`config_secuencia.h`) elige el orden de la sujeción:

- `true` (por defecto): CONTACTO → centrado con presión ligera → APRETANDO (compresión final).
- `false`: CONTACTO → compresión → centrado, como en `PruebaSecuenciaManual`.

| Etapa | Acción | Termina cuando | LED |
|---|---|---|---|
| INICIALIZANDO / EMERGENCIA | Espera de parada (25 s / 20 s) | LISTO; START o `I` autoriza HOME | amarillo titila → verde titila |
| REFERENCIANDO | M3 a HOME; después M1 y M2 a HOME | Los tres HOME confirmados | amarillo fijo |
| ESPERANDO START | — | START o `I` | verde fijo |
| SUJETANDO | M1 y M2 cierran al 25 % hasta CONTACTO (con `false`, además comprimen) | Ambos CONTACTOS confirmados + 300 ms | amarillo fijo |
| POSICIONANDO | Centrado por PID (25 % máx.) + desfase de +1 mm a la derecha | Error ≤ 2 mm por mordaza y separación ≤ 5 mm, en ≤ 10 s | amarillo fijo |
| APRETANDO | Cada mordaza avanza hasta 10 mm al 40 % | Objetivo o 1 s sin cuentas, ambos CONTACTOS + 300 ms | amarillo fijo |
| ARRANCANDO SIERRA | Contactor ON | 2 s | amarillo fijo |
| AVANZANDO CORTE | M3 a END: rampa 0→65 % en 500 ms | END confirmado | amarillo fijo |
| RETORNANDO | M3 a HOME al 65 % | HOME confirmado; contactor OFF | amarillo fijo |
| APAGANDO SIERRA | Espera de parada del disco | 20 s | amarillo titila |
| LIBERANDO | M1 y M2 a HOME | Mordazas en HOME sin CONTACTO | amarillo fijo |
| CICLO COMPLETADO | — | Retirar pieza; START inicia otro ciclo | azul fijo |

Si las mordazas agotan su recorrido sin CONTACTO, vuelven a HOME (ABRIENDO SIN PIEZA) y regresan a ESPERANDO START.

## LED de la placa (GPIO48)

| LED | Significado |
|---|---|
| Verde fijo | Lista: pulse START para cortar |
| Verde titilando | Pulse START: para HOME o para reanudar una pausa |
| Amarillo fijo | Trabajando: no intervenir |
| Amarillo titilando | Espere: tiempo de seguridad |
| Azul fijo | Retire la pieza |
| Rojo fijo | Emergencia activa |
| Rojo titilando | Fallo: revisar el monitor y pulsar START dos veces para reiniciar |

El detalle de cada fallo se lee en el monitor serie (`Q`).

## STOP

- Primer `S`/`X`: pausa con todo apagado (LED verde titilando). START o `I` reanuda.
- Segundo `S`/`X`: pide volver a HOME. Tras 25 s (amarillo titilando → verde titilando), START o `I` confirma.

## Protecciones relacionadas con la pieza

| Situación | Respuesta |
|---|---|
| CONTACTO falta durante la compresión | PWM=0 mientras falta; fallo si dura ≥ 200 ms |
| CONTACTO falta durante el centrado | Ambas mordazas en pausa (PWM=0); fallo si una mordaza lo pierde 2 s seguidos |
| CONTACTO falta con la pieza sujeta (apriete y sierra) | Fallo si una mordaza lo pierde 2 s seguidos |
| Mordaza sin avance durante el centrado, **con** CONTACTO | Se detiene y se evalúa la tolerancia de ±2 mm |
| Mordaza sin avance durante el centrado, **sin** CONTACTO | Fallo: "Sin avance y sin CONTACTO durante el centrado" |
| Centrado que no termina | Fallo a los 10 s |
| Movimiento de las mordazas con la sierra en marcha | Solo registro: línea `DERIVA` al terminar |

## Parámetros principales

| Parámetro | Valor | Archivo |
|---|---|---|
| PID M1/M2: Kp, Ki, Kd | 1,6468 %/mm; 2,6383 %/(mm·s); 0,076832 %·s/mm | `config_eje.h` |
| Periodo de control | 10 ms (fallo si supera 100 ms) | `config_eje.h` |
| PWM (LEDC) | 20 kHz, 10 bits | `config_eje.h` |
| Aproximación / límite PID con pieza | 25 % / 25 % | `config_eje.h` |
| Pausa de la mordaza adelantada (centrado) | pausa > 1 mm, reanuda < 0,5 mm | `config_geometria.h` |
| Compresión | 10 mm, 40 %, máximo 5 s; filtro de CONTACTO 200 ms | `config_secuencia.h` |
| Desfase del centro | +1 mm (derecha) | `config_secuencia.h` |
| Centrado: parada del motor / aceptación / separación / plazo | 0,25 mm / ±2 mm / ±5 mm / 10 s | `config_secuencia.h` |
| Pérdida de CONTACTO admitida con pieza | 2 s | `config_secuencia.h` |
| M3 avance y retorno | 65 % con rampa de 500 ms | `config_eje.h` |

## Comandos del monitor

| Entrada | Función |
|---|---|
| `I` | Igual que START: autoriza HOME, inicia el ciclo, reanuda una pausa o confirma el HOME pedido con STOP |
| `S` o `X` | STOP (ver arriba) |
| `Q` / `E` | Estado / diagnóstico con pines y encoders |
| `?` | Ayuda |

En BLOQUEADO: revisar la causa con `Q`; dos START o dos `I` en menos de 5 s reinician la ESP32 con el contactor OFF.

## Cambios respecto a PruebaSecuenciaManual

1. Sin perfiles: un solo entorno `cortadora`.
2. Desfase de centrado como una sola constante `DESFASE_CENTRO_MM=+1` (equivale a M1 +1 / M2 −1 mm).
3. `stalledHeld`: una mordaza sin avance durante el centrado se acepta solo si su CONTACTO está activo.
4. Dos tolerancias de centrado: parada del motor a 0,25 mm y aceptación a ±2 mm (antes una sola de 2 mm, que impedía ejecutar el desfase de 1 mm).
5. Plazo total de centrado de 10 s.
6. Durante el centrado se supervisa el CONTACTO (pausa y fallo a los 2 s).
7. Filtro de 200 ms para la pérdida de CONTACTO durante la compresión.
8. Orden de sujeción seleccionable; por defecto se centra antes de comprimir (etapa APRETANDO).
9. Se eliminaron las confirmaciones manuales con `N`; todos los tiempos son fijos.
10. START físico confirma el HOME pedido con doble STOP.
11. Registro de deriva de las mordazas durante la sierra y de la lectura de M3 antes de poner cero en HOME.
12. LED simplificado (tabla anterior).
13. Como en `PruebaSujecionDosMotores`: aproximación al CONTACTO al 25 % y pausa de la mordaza adelantada durante el centrado (la línea `CENTRADO` informa cuántas pausas hubo).
14. Registro del monitor en `logs/` y resumen de tiempos por ciclo (`CICLO #n`).
