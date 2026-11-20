# Sistema automatizado para el corte de ladrillos

Trabajo de grado de Ingeniería Mecatrónica, Pontificia Universidad Javeriana (Bogotá).

Prototipo que sujeta, centra y transporta un ladrillo frente a una sierra fija. Lo controla una ESP32-S3 con tres motores BLDC: dos mordazas con PID de posición (M1, M2) y un eje de transporte (M3).

**Autores:** _completar_ · **Director:** _completar_ · **Fecha:** noviembre de 2026

## Estructura

| Carpeta | Contenido |
|---|---|
| [01_Documentacion](01_Documentacion/) | Documento de tesis y referencias |
| [02_Diseno_Mecanico](02_Diseno_Mecanico/) | Planos (PDF y SolidWorks) y modelos CAD |
| [03_Electronica](03_Electronica/) | PCB (Altium), Gerber y esquemático |
| [04_Programacion](04_Programacion/) | **Firmware final** y versiones de prueba |
| [05_Control_y_Simulacion](05_Control_y_Simulacion/) | Modelos, control y simulaciones (MATLAB) |
| [06_Pruebas_y_Resultados](06_Pruebas_y_Resultados/) | Protocolo y registro de pruebas |
| [07_Presentacion](07_Presentacion/) | Presentación de la sustentación |

Firmware final: [`04_Programacion/ESP32_PlatformIO/Firmware_Cortadora`](04_Programacion/ESP32_PlatformIO/Firmware_Cortadora/)
