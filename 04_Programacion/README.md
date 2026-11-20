# Programación (ESP32-S3, PlatformIO)

## Firmware final
**[`ESP32_PlatformIO/Firmware_Cortadora`](ESP32_PlatformIO/Firmware_Cortadora/)**: versión final y comentada del firmware de la cortadora. Cada archivo empieza con una cabecera que explica su función; `src/main.cpp` describe el ciclo completo. Su `README.md` detalla la compilación, las etapas del ciclo, las protecciones y los comandos del monitor serie.

Compilar y cargar (desde la carpeta del proyecto):
```
pio run -e cortadora -t upload
pio device monitor
```

> Nota: en `include/config_secuencia.h`, `COMPRIMIR_DESPUES_DE_CENTRAR = true` (centrar y luego comprimir). Los ensayos de corte reportados en la tesis se hicieron con `PruebaSecuenciaManual`, que comprime y luego centra; ese orden equivale a `false`.

## Versiones de prueba
| Carpeta | Uso |
|---|---|
| `Versiones_Experimentales/PruebaSecuenciaManual` | Secuencia completa usada en los ensayos de corte (perfil `banco-contactor`). |
| `Versiones_Experimentales/PruebaSujecionDosMotores` | Primera versión de sujeción y centrado con dos mordazas. `Comandos-y-fallos_Tesis.txt` documenta comandos y mensajes de fallo. |
| `Pruebas_Individuales/CaracterizacionMotorLineal` | Ensayos de identificación de los motores (escalones de PWM) usados para el modelo y el PID. |

Los README de las versiones de prueba describen el estado de cada una en su momento y pueden no coincidir con el firmware final.
