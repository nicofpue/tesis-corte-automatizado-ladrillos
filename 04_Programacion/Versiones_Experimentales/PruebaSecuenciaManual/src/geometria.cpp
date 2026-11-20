#include "geometria.h"
#include <math.h>
float faceGap(const GeometryConfig& c,float x1,float x2) { return c.homeFaceGapMm()-x1-x2; }
bool separationTargets(const GeometryConfig& c,float gap,float travel1,float travel2,float& target1,float& target2) {
    // Dos objetivos absolutos propios: ambas caras a gap/2 del centro FIJO.
    // No exigir iguales cuentas, iguales desplazamientos ni relacion de recorridos.
    target1=c.leftHomeToCenterMm-gap/2;
    target2=c.rightHomeToCenterMm-gap/2;
    return isfinite(gap)&&gap>0&&isfinite(target1)&&isfinite(target2)&&
        target1>=0&&target1<=travel1&&target2>=0&&target2<=travel2;
}
GripGeometry estimatedGrip(const GeometryConfig& c,float x1,float x2) {
    GripGeometry g;
    // Coordenada global: centro fijo=0, derecha positiva. Caras retroceden al comprimir.
    g.left=-c.leftHomeToCenterMm+x1-c.contactOffsetMm*c.leftContactShare;
    g.right=c.rightHomeToCenterMm-x2+c.contactOffsetMm*(1-c.leftContactShare);
    g.width=g.right-g.left;g.center=(g.left+g.right)/2;
    return g;
}
bool translationRange(float x1,float x2,float travel1,float travel2,ShiftRange& r) {
    if (!isfinite(x1)||!isfinite(x2)||!isfinite(travel1)||!isfinite(travel2)||
        x1<0||x1>travel1||x2<0||x2>travel2) return false;
    // x1'=x1+delta, x2'=x2-delta; ambos objetivos deben quedar en sus recorridos.
    r.minimum=fmaxf(-x1,x2-travel2);
    r.maximum=fminf(travel1-x1,x2);
    return r.minimum<=r.maximum;
}
