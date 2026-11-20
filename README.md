# Sistema automatizado para el corte de ladrillos

Trabajo de grado de Ingeniería Mecatrónica, Pontificia Universidad Javeriana (Bogotá).

Prototipo que sujeta, centra y transporta un ladrillo frente a una sierra fija. El operario solo coloca y retira la pieza. Lo controla una ESP32-S3 con tres motores BLDC: dos mordazas con control PID de posición (M1 y M2) y un eje de transporte (M3).

**Autores:** _completar_ · **Director:** _completar_ · **Fecha:** noviembre de 2026

## Empiece aquí

| Si busca… | Vaya a |
|---|---|
| **El firmware final** | [`04_Programacion/ESP32_PlatformIO/Firmware_Cortadora`](04_Programacion/ESP32_PlatformIO/Firmware_Cortadora/) |
| Los planos en PDF (sin SolidWorks) | [`02_Diseno_Mecanico/Planos/PDF`](02_Diseno_Mecanico/Planos/PDF/) |
| El modelo 3D completo (SolidWorks) | [`02_Diseno_Mecanico/Modelos_CAD`](02_Diseno_Mecanico/Modelos_CAD/) |
| El esquemático y la PCB | [`03_Electronica`](03_Electronica/) |
| Los datos de las pruebas | [`06_Pruebas_y_Resultados`](06_Pruebas_y_Resultados/) |

## Estructura

- [01_Documentacion](01_Documentacion/): documento de tesis y referencias.
- [02_Diseno_Mecanico](02_Diseno_Mecanico/): planos (PDF y SolidWorks) y modelos CAD.
- [03_Electronica](03_Electronica/): proyecto Altium de la PCB, Gerber y esquemático en PDF.
- [04_Programacion](04_Programacion/): firmware final, versiones de prueba y caracterización de motores.
- [05_Control_y_Simulacion](05_Control_y_Simulacion/): identificación de motores, diseño del PID y simulaciones (MATLAB).
- [06_Pruebas_y_Resultados](06_Pruebas_y_Resultados/): protocolo y registro de pruebas.
- [07_Imagenes_y_Diagramas](07_Imagenes_y_Diagramas/): fotografías y diagramas.

## Datos principales del prototipo

- Controlador: ESP32-S3-DevKitC-1 (framework Arduino, PlatformIO).
- Motores: 3 × BLDC 57BLDC75E (24 V, 70 W) con drivers ZS-X11H; husillos T8 de 8 mm/vuelta; M3 con reductor 10:1.
- Control: PID de posición para M1/M2 cada 10 ms (ubicación de polos sobre el modelo identificado); M3 en lazo abierto con rampa.
- Sierra: 2500 W, disco de 14", conmutada por contactor.
