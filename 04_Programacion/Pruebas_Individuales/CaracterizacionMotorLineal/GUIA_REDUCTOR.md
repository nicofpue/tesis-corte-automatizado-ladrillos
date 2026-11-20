# Caracterizacion individual V3

Seleccionar exactamente MOTOR_M1, MOTOR_M2 o MOTOR_M3 en include/config.h.
M1/M2: reduccion 1. M3: reduccion 10, A/R no disponibles.
Cada bloque tiene sus propios pines, polaridades, sentidos, escalas y limites.
Pines contrastados con Control3Motores/include/config_eje.h; verificar cableado real.

Antes de habilitar: configurar MAX_TEST_PWM y MAX_TRAVEL_MM (inicialmente cero)
y AXIS_CONFIG_VERIFIED=true en el eje elegido. No se consideran validados los
antiguos 90 % ni 1000 mm. Elegir limites con margen de frenado y compresion admisible.
Comprobar fisicamente GPIO, niveles de 3.3 V, resistencias y modo de entrada,
HOME/END/CONTACTO independientes, niveles activos, DIR_POSITIVE_LEVEL,
CLAMP_DIR/OPEN_DIR, ENCODER_SIGN, PPR x1 y avance del husillo.
Entradas activas LOW: 3.3 V en reposo, 0 V presionadas.
M1/M2 INPUT_PULLUP; M3 INPUT con resistencias externas, como Control3Motores. CONTACTO: M1 GPIO 6, M2 GPIO 8; M3 sin CONTACTO.
Validar tambien tiempos maximos (15 s), confirmacion de liberacion (20 ms)
y margen de inercia. Encoder x1: RISING de A, lectura de B para sentido.

## Comandos (terminar con Enter)

- ?: ayuda y limites; Q: encoder y HOME/END/CONTACTO.
- Z: cero local, no homing. X: cancelar (tambien por caracter).
- S,PWM,TIEMPO_MS,DIR: escalon; fin normal TIEMPO_COMPLETADO.
- C,PWM,CUENTAS,DIR: desplazamiento en cuentas; fin CUENTAS_COMPLETADAS.
- A,PWM,TIEMPO_MAX_MS,DIR: solo CLAMP_DIR; fin normal CONTACTO.
- R,PWM,TIEMPO_MAX_MS: solo OPEN_DIR; exige CONTACTO activo y finaliza
  tras 20 ms continuos inactivo, con motivo CONTACTO_LIBERADO.

PWM es porcentaje; DIR vale 1 o -1. A/R exigen PWM mayor que cero.
A/R agotados terminan TIMEOUT, nunca como completados.
A y S/C rechazan CONTACTO activo y se detienen si se activa.
R requiere orden explicita y mantiene HOME, END, recorrido, tiempo y X.
Si HOME o END esta activo, tambien se rechaza R: no sirve para salir de finales.
R no hace homing, no pone cero ni prepara otro ciclo; comprobar posicion
fisicamente antes de repetir. # LISTO solo significa fin de transmision.
Esperar # LISTO antes de enviar otra orden. No hay movimientos al arrancar.
No hay PID, coordinacion, sierra ni parada por falta breve de cuentas.
La comprobacion de reposo inicial y quiet final no son detectores de atasco.

## Datos

Muestreo nominal 5000 us. Muestras y eventos en RAM; envio tras parada y
700 ms de observacion. Sin streaming durante movimiento.
Se conservan exactamente las diez columnas:
intento,t_us,pwm,dir,counts,delta_counts,x_mm,v_mm_s,rpm_salida,vueltas_salida
Se conservan sus formulas, orden y unidades; delta_counts sigue siendo
el desplazamiento acumulado desde inicio, como en el firmware original.
# START, # CONFIG (motor/version), # EVENT (on_us/off_us), # END y # LISTO.
# INPUT,intento=N,t_us=T,input=HOME|END|CONTACTO,activo=0|1
incluye estado inicial y cambios observados por sondeo durante PRE/ON/COAST;
T relativo al ensayo, en microsegundos. No captura pulsos mas cortos que el sondeo.
Los # INPUT se envian despues de las muestras, en orden temporal propio.
512 eventos; al llenarse durante PRE/ON se detiene con BUFFER_INPUT_LLENO.
# END indica overflow agregado e input_overflow especifico; cualquier overflow
invalida la integridad del log, incluso si ocurre durante COAST.
El buffer de muestras conserva BUFFER_LLENO y overflow; no sobrescribe datos.
No se modifica MATLAB.

## Compilacion sin carga

PowerShell en la carpeta del proyecto:
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
No usar upload para esta revision. La validacion fisica queda pendiente.
