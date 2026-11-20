# Prueba de la secuencia automática de corte

Proyecto PlatformIO para la ESP32-S3-DevKitC-1, basado en `PruebaSujecionDosMotores`. Reutiliza el encoder, los finales, el PID, la compensación entre mordazas, la búsqueda de HOME/END de M3 y el panel del proyecto base.

## Perfil de banco con contactor real

**banco-contactor** es el único entorno de PlatformIO. Compila y carga con:

```powershell
pio run -e banco-contactor -t upload
```

El banco acciona el contactor y mueve M1, M2 y M3. Úsalo con la sierra físicamente desconectada.
El monitor identifica el perfil con `Q` o `?`.

- Durante el centrado se omiten la pausa por pérdida de contacto y la pausa del eje adelantado. Se conserva el ajuste continuo de sincronización.
- El resultado se acepta con hasta ±2 mm por mordaza y ±5 mm de diferencia frente a la separación objetivo, que incorpora 1 mm de retroceso de M1 hacia HOME y 1 mm de avance de M2 hacia END. Se omiten el plazo de 10 s y la espera adicional de 100 ms de la secuencia. Los límites y diagnósticos propios de cada motor siguen activos.
- Se requieren los dos contactos antes de accionar el contactor. Después, el ciclo se bloquea si una misma mordaza pierde contacto durante 2 s continuos. El diagnóstico indica la duración observada por mordaza. La deriva por encoder después de centrar se omite en este perfil.
- M1 y M2 se acercan al **30 %**. Cada mordaza se detiene en su primer CONTACTO y, al confirmarlo, intenta avanzar hasta **10 mm adicionales al 40 %**, dentro de su recorrido configurado.
- Cada mordaza corta su PWM al alcanzar su objetivo o pasar **1 s** sin nuevas cuentas con PWM activo. El monitor muestra el avance medido de cada eje. END, perder CONTACTO durante la compresión o agotar **5 s** bloquean el ciclo. El centrado comienza cuando ambas han terminado y los dos contactos están confirmados.
- M3 dispone de hasta 150 s para llegar a END y 120 s para llegar a HOME. El límite por falta de cuentas con PWM activo sigue siendo de 1 s.

## Prueba de la secuencia

1. Abre la carpeta `PruebaSecuenciaManual` en VS Code con PlatformIO. Compila con **Build** y carga con **Upload** cuando el banco esté preparado.
2. Comprueba que la sierra esté desconectada, que el recorrido esté libre y que la parada física funcione.
3. Abre el monitor serie a **115200 baudios**. Envía `Q` y comprueba que los sensores cambian de estado según lo esperado y que `STOP/EMERG=0` en reposo. En el código base, M3 usa **HOME GPIO7** y **END GPIO15**, ambos activos en LOW. Compruébalos por separado contra el cableado antes de ordenar HOME.
4. Envía un comando por vez con Enter. Tras LISTO, usa START o `I` para iniciar la referencia. Cuando el sistema llegue a **ESPERANDO START**, pulsa START o envía `I` para ejecutar un ciclo completo.

| Estado actual | Orden | Resultado |
|---|---|---|
| INICIALIZANDO o EMERGENCIA, listo | START físico o `I` | Con el circuito físico restablecido y el disco inmóvil, busca HOME de M3 (hasta 120 s). Después abre M1 y M2 hasta sus HOME. El contactor se mantiene OFF. |
| REFERENCIANDO, listo | Automático | Entra en ESPERANDO START. |
| ESPERANDO START | START físico o `I` | M1 y M2 cierran en paralelo al 30 % de PWM. Tras confirmar su CONTACTO, cada uno intenta la compresión al 40 %. |
| ABRIENDO SIN PIEZA | Automático | Si se agota el recorrido sin ningún CONTACTO, ambas mordazas vuelven a HOME con la sierra apagada. Después vuelve a ESPERANDO START y requiere una orden nueva. |
| SUJETANDO, listo | Automático | Calcula el centro estimado y centra la pieza. |
| POSICIONANDO, listo | Automático | Una vez validado el centrado y confirmados los contactos, ordena contactor ON. |
| ARRANCANDO SIERRA | Automático tras 2 s | M3 avanza hasta END. |
| AVANZANDO CORTE, END confirmado | Automático | M3 retorna a HOME. Al confirmar HOME, se ordena OFF. |
| APAGANDO SIERRA | Automático tras 20 s | M1 y M2 vuelven a HOME. |
| LIBERANDO, listo | Automático | Marca CICLO COMPLETADO. |
| CICLO COMPLETADO | START físico o `I` | Inicia otro ciclo tras el retiro o cambio manual de la pieza. |

## Monitor y pulsadores

| Entrada | Función |
|---|---|
| `I` | Autoriza HOME al encender o tras una emergencia. En ESPERANDO START inicia la sujeción. En PAUSADO reanuda o confirma el HOME solicitado, según lo que indique el monitor. |
| `N` | Confirma el arranque o la apertura si se selecciona confirmación manual. El ciclo normal avanza por sí solo. |
| `S` o `X` | Primer STOP para pausar. Una orden nueva de STOP durante la pausa solicita HOME. |
| `Q` | Muestra etapa, ejes, sensores, panel y tiempo de parada restante cuando corresponde. |
| `E` | Añade pines, ajustes PWM y lecturas de encoder. |
| `?` | Muestra la ayuda. |
| START físico, GPIO16 | Normalmente abierto de fábrica. Una pulsación nueva inicia un ciclo en ESPERANDO START o CICLO COMPLETADO, o reanuda una pausa recuperable. En INICIALIZANDO o EMERGENCIA, una pulsación después de LISTO autoriza HOME. Si el STOP físico se habilita, primero debe estar liberado. |
| STOP físico, GPIO21 | Deshabilitado temporalmente. Usa `S` o `X` en el monitor. Para volver a leer este pulsador, cambia `physicalStopEnabled` a `true` en `config_panel.h` y carga el firmware. |
| EMERGENCIA, GPIO43 | Normalmente cerrada. El circuito externo corta la alimentación de la máquina. Si el ESP32 conserva energía, desactiva sus salidas y entra en EMERGENCIA. Para recuperar, restablece el circuito y pulsa START o envía `I` cuando se indique LISTO. |

Las órdenes de banco `D`, `C`, `T`, `ON` y `H3` pertenecen al proyecto de referencia. En este proyecto, sus funciones de movimiento se ejecutan en las etapas que corresponden.

## LED RGB de la placa

Por defecto, el GPIO48 usa la misma señalización de `PruebaSujecionDosMotores`. Rojo fijo mientras la entrada de emergencia esté activa y confirmada por 30 ms, amarillo fijo con orden de contactor ON y apagado en los demás casos. El rojo de emergencia tiene prioridad. La secuencia de parada del firmware se activa desde la lectura directa y conserva su recuperación aunque el rojo se apague al liberar el pulsador.

Con `stageIndicatorsEnabled=true` en `config_panel.h`, se añaden las señales de etapa de esta tabla.

| Señal | Estado |
|---|---|
| Verde fijo | LISTO para una confirmación o ESPERANDO START. |
| Azul intermitente | Ejes en movimiento o confirmando un final. |
| Amarillo fijo | Orden de contactor ON. Tiene prioridad sobre el azul. |
| Amarillo intermitente | Espera de parada, pausa por STOP u otra espera sin movimiento. |
| Rojo fijo | Entrada de emergencia confirmada, bloqueo o posicionamiento incompleto. |

El RGB del GPIO48 depende de la alimentación de la placa. El amarillo fijo indica la orden eléctrica del contactor.

## Ajustes editables

| Archivo | Ajustes |
|---|---|
| `include/config_secuencia.h` | Avance, PWM y tiempo de compresión, tolerancias de banco y tiempos del ciclo. |
| `include/config_eje.h` | GPIO, recorridos, PID común de mordazas en `PidConfig`, PWM de cierre en `mordazaConfig()` y ajustes de M3. |
| `include/config_panel.h` | GPIO y polaridades. Con `startNormallyClosed=true` se selecciona START NC y STOP NO. |
| `include/config_geometria.h` | Geometría, reparto de la corrección, estabilización y ajuste de sincronización. |

El centrado usa el cálculo de `C` del proyecto de referencia. Espera 300 ms desde el último fin de compresión y verifica que ambos contactos estén confirmados. La línea SUJETA muestra la apertura corregida, la separación de los carros y el centro estimado. A continuación toma las posiciones actuales, calcula objetivos opuestos y comprueba el recorrido. Durante el movimiento revisa el desfase después de actualizar cada eje. La línea CENTRADO repite las medidas con las posiciones finales. Si el centro estimado está a 0,15 mm o menos del centro fijo, la etapa se completa con las mordazas quietas.

La apertura estimada suma 34,344 mm a la separación de los carros. El cálculo usa un reparto 50/50 para estimar el centro de la pieza. El centrado usa hasta 25 % de PWM por mordaza y corta el PWM al terminar.

## Paradas y tiempos

- Durante el arranque de la sierra, una pérdida breve de HOME de M3 inhibe el avance. Si HOME vuelve a quedar activo durante 30 ms, se permite continuar. Si permanece inactivo durante 1 s o la señal fluctúa durante 1,2 s sin estabilizarse, se bloquea el ciclo y se ordena OFF.

- M3 busca END y HOME con una rampa lineal de **0 a 65 % en 500 ms**. Después mantiene el 65 % hasta detectar END o HOME. El sensor HOME sigue determinando el punto de parada. Al detectar el final, STOP, emergencia o un fallo, corta el PWM de inmediato. Los parámetros de rampa y aproximación se editan en `config_eje.h`.
- Al autorizar el arranque se muestran los parámetros de M3. Si M3 tiene PWM aplicado durante **1 s** sin cambio en las cuentas del encoder, se bloquea el ciclo y se ordena apagar el contactor. El monitor muestra automáticamente la causa, PWM previo, cuentas, tiempo sin cambio, tiempo de la maniobra y estados HOME/END. `Q` y `E` permiten volver a ver el fallo de M3. Los bloqueos durante la fase de sierra incluyen la etapa y las señales de contacto de las mordazas.

- El primer `S` o `X` deja el ciclo en PAUSADO, con PWM en cero y contactor OFF. Pulsa START o envía `I` para reanudar una etapa recuperable.
- Si se pausó durante el arranque de la sierra con M3 en HOME, se espera el tiempo de parada y un nuevo START o `I` reanuda desde POSICIONANDO. El contacto y la posición se comprueban antes de volver a ordenar ON. Un avance de corte o retorno interrumpido exige recuperación a HOME.
- Un segundo `S` o `X` solicita HOME. Espera 25 s desde la pausa, verifica que el disco esté inmóvil y el recorrido libre y confirma con `I`. M3 vuelve primero y luego abren M1 y M2, con el contactor OFF.
- La pérdida de contacto con la pieza sujeta bloquea el ciclo. Al intentar reanudar el centrado o arranque se comprueban nuevamente los contactos y HOME de M3.
- En BLOQUEADO o POSICIONAMIENTO INCOMPLETO, revisa la causa con `Q` o `E`. Después pulsa START dos veces, soltándolo entre pulsaciones, o envía `I` dos veces con Enter entre comandos, dentro de 5 s. El ESP32 reinicia con el contactor OFF. Tras la espera inicial de 25 s, un nuevo `I` autoriza HOME. Durante una emergencia se aplica el rearme físico descrito a continuación.
- Si aparece END o se alcanza el recorrido configurado sin haber detectado ningún contacto, ambas mordazas vuelven a HOME automáticamente. Si hubo contacto parcial, el ciclo se bloquea para revisar la pieza.
- Al detectar EMERGENCIA se cancela el ciclo. Si ya estaba BLOQUEADO, el firmware entra en EMERGENCIA y sigue su procedimiento de recuperación. Los fallos activos de los ejes impiden volver a HOME hasta resolverlos. Restablece la parada en el circuito físico, comprueba que el disco esté inmóvil y que M3 se pueda retirar con seguridad. Si hay una pieza o un disco atrapado, aplica primero el procedimiento físico de recuperación con la máquina aislada. Tras los 20 s y el mensaje LISTO, una pulsación nueva de START o un `I` autoriza volver a HOME con el contactor OFF. M3 vuelve primero y luego abren las mordazas. Si la emergencia también apagó el ESP32, al encender aplica una espera de 25 s en INICIALIZANDO. [Rearme de la parada de emergencia](https://www.pilz.com/en-INT/support/faq/standards/articles/180045)
- El arranque real espera **2 s** desde la orden ON y la apertura normal espera **20 s** desde la orden OFF, según la verificación en banco. En EMERGENCIA la cuenta de 20 s empieza al detectar la activación. Si el ESP32 se apaga, cuenta 25 s desde su arranque. `ARRANQUE_REAL_MS=0` activa la confirmación manual del arranque con `N`, con un plazo de 20 s. `PARADA_REAL_VERIFICADA_MS=0` exige `N` para liberar después de la espera.
- Los tiempos habilitan las transiciones del firmware. El equipo carece de sensor de giro y realimentación del contactor. El circuito de parada y los resguardos dan la protección física. [Guía de seguridad para maquinaria de carpintería de OSHA](https://www.osha.gov/Publications/osha3157.pdf)

