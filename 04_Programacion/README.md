# Programación (ESP32-S3, PlatformIO)

| Carpeta | Contenido |
|---|---|
| **`ESP32_PlatformIO/Firmware_Cortadora`** | **Firmware final**, comentado. Ver su `README.md`. |
| `Versiones_Experimentales/PruebaSecuenciaManual` | Secuencia usada en los ensayos de corte. |
| `Versiones_Experimentales/PruebaSujecionDosMotores` | Primera versión de sujeción y centrado. |
| `Pruebas_Individuales/CaracterizacionMotorLineal` | Identificación de los motores para el diseño del PID. |

Compilar y cargar el firmware final:
```
pio run -e cortadora -t upload
pio device monitor
```
