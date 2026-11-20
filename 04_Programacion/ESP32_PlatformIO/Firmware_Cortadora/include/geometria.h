#pragma once
// =====================================================================
// geometria.h - Calculos de la pieza a partir de la posicion de las mordazas
// ---------------------------------------------------------------------
// Coordenada global: centro fijo (plano del disco) = 0, derecha positiva.
// x1, x2 = avance de cada mordaza desde su HOME, en mm.
// =====================================================================
#include "config_geometria.h"
// Separacion entre las caras de las mordazas: aL + aR - x1 - x2.
float faceGap(const GeometryConfig& c, float x1, float x2);
// Objetivos que dejan ambas caras a la misma distancia del centro fijo.
bool separationTargets(const GeometryConfig& c, float gap, float travel1, float travel2, float& target1, float& target2);
// Caras, ancho y centro ESTIMADOS de la pieza sujeta.
struct GripGeometry { float left=0,right=0,width=0,center=0; };
GripGeometry estimatedGrip(const GeometryConfig& c, float x1, float x2);
// Traslado comun admisible sin sacar a ninguna mordaza de su recorrido.
struct ShiftRange { float minimum=0,maximum=0; };
bool translationRange(float x1,float x2,float travel1,float travel2,ShiftRange& range);
