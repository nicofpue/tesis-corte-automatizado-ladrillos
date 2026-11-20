# Diseño mecánico

## Planos
- **`Planos/PDF/`**: planos listos para ver o imprimir, por subsistema (Electronica, Ensamble, Estructura, Sujecion).
- **`Planos/Planos_SolidWorks.zip`**: los mismos planos en formato fuente (`.SLDDRW`), los ensambles en explosión y la plantilla de rótulo PUJ (`PlantillaPUJ_A4.slddrt`).

## Modelos CAD (SolidWorks)
| Archivo | Contenido | Archivo principal |
|---|---|---|
| `Modelos_CAD/CAD_Diseno_Final.zip` | Diseño final completo: estructura, sujeción, sierra, sistema de riego, guardas y electrónica | `CAD_Cortadora_Ladrillos.SLDASM` |
| `Modelos_CAD/CAD_Prototipo.zip` | Versión del prototipo de banco (pruebas) | `Prototipo/Pruebas2/CAD_Cortadora_Ladrillos.SLDASM` |

Los dos modelos están divididos en volúmenes de 20 MB (`.z01`, `.z02`, … + `.zip`) para respetar el límite de tamaño de GitHub. Para abrir: descargar **todos** los volúmenes de un modelo en la misma carpeta, descomprimir el archivo `.zip` con 7-Zip o WinRAR (el explorador de Windows no abre archivos divididos) conservando las carpetas, y abrir el ensamble principal en SolidWorks. Si se mueven piezas fuera de su carpeta, el ensamble pierde las referencias.

No se incluyeron los resultados de simulación de SolidWorks Simulation (`.CWR` y archivos auxiliares), los archivos temporales (`~$*`) ni un paquete de componente descargado de 128 MB que supera el límite de GitHub.
