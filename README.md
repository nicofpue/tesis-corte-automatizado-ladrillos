# tesis-corte-automatizado-ladrillos

Diseño y desarrollo de un prototipo automatizado para el corte de ladrillos de mampostería

Trabajo de grado de Ingeniería Mecatrónica. El prototipo usa un ESP32-S3 y una máquina de estados para coordinar el proceso de corte.

## Estructura del repositorio

- [01_Documentacion](01_Documentacion/) — Documento de tesis, bibliografía y entregables.
- [02_Diseno_Mecanico](02_Diseno_Mecanico/) — Planos, modelos CAD y cálculos mecánicos.
- [03_Electronica](03_Electronica/) — Esquemáticos, PCB y conexiones eléctricas.
- [04_Programacion](04_Programacion/) — Firmware ESP32-S3, pruebas independientes y prototipos de código.
- [05_Control_y_Simulacion](05_Control_y_Simulacion/) — Caracterización de motores BLDC, modelado, control y simulaciones.
- [06_Pruebas_y_Resultados](06_Pruebas_y_Resultados/) — Datos experimentales, gráficas e informes.
- [07_Imagenes_y_Diagramas](07_Imagenes_y_Diagramas/) — Fotografías, diagramas de estados y diagramas del sistema.
- [08_Presentacion](08_Presentacion/) — Diapositivas y materiales de sustentación.

## Firmware y pruebas

Conserva cada proyecto PlatformIO independiente en una carpeta con su propio `platformio.ini`. Usa entornos en ese archivo para variantes que comparten código. Guarda las pruebas autónomas en `04_Programacion/Pruebas_Individuales/`.

En cada experimento registra la fecha, el objetivo, la configuración del prototipo, el commit del firmware y la ubicación de los datos originales.
