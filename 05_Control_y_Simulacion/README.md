# Control y simulación (MATLAB)

Requiere MATLAB con Control System Toolbox. Ningún script se conecta a los motores.

## M1_M2_Identificacion_y_PID
Identificación de los motores de las mordazas y diseño del PID de posición.

1. `SimulacionModeloDinamico_Revisado.m`: identifica Kv, τ y zona muerta a partir de `pruebas_motor.txt` y guarda `Modelo_Eje1.mat`.
2. `DisenoPID_Posicion.m`: calcula Kp, Ki y Kd por asignación de polos y simula el lazo con el periodo (10 ms), el PWM máximo (20 %) y el filtro derivativo (0.03 s) del firmware.

Con Kv = 3.99 (mm/s)/% y τ = 0.020 s se obtienen las ganancias del firmware: Kp = 1.6468, Ki = 2.6383, Kd = 0.076832.

## M3_Identificacion
Identificación del motor de transporte (M3, reductor 10:1).

En el ciclo, M3 trabaja en lazo abierto entre sus finales de carrera HOME y END (PWM 65 % con rampa de 500 ms). El encoder supervisa que el motor avance y, al volver a HOME, se registra su lectura para comprobar que no pierde cuentas.

`Modelado_M3.m`: identifica el modelo por sentido a partir de `PruebasMotor3.txt` y guarda `Modelo_M3_DIR1.mat`, `Modelo_M3_DIRmenos1.mat` y `Resultados_M3/`.

## Modelo_Dinamico_Maquina
Modelo mecánico de la sujeción: M1, M2 y el ladrillo. Ejecutar `EjecutarModelos.m`; `VerificarModelos.m` hace pruebas numéricas del modelo. Los parámetros de `ParametrosMaquina.m` son supuestos, por lo que los resultados son ilustrativos.
