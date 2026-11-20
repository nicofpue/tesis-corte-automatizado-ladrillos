#pragma once
#include "config_geometria.h"
float faceGap(const GeometryConfig& c, float x1, float x2);
bool separationTargets(const GeometryConfig& c, float gap, float travel1, float travel2, float& target1, float& target2);
struct GripGeometry { float left=0,right=0,width=0,center=0; };
GripGeometry estimatedGrip(const GeometryConfig& c, float x1, float x2);
struct ShiftRange { float minimum=0,maximum=0; };
bool translationRange(float x1,float x2,float travel1,float travel2,ShiftRange& range);
