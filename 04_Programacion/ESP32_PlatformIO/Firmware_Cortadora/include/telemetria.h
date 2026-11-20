#pragma once
// =====================================================================
// telemetria.h - Mensajes por el monitor serie sin bloquear
// ---------------------------------------------------------------------
// Los mensajes se guardan en una cola de 16 lineas y se envian por partes
// en cada vuelta de loop(): escribir por el USB nunca detiene el control.
// =====================================================================
#include "eje.h"
// Devuelve false si la cola esta llena; el informe reintenta ese paso.
bool printReply(const char* text);
bool printStatus(const Axis& axis);
bool printEncoder(const Axis& axis);
bool printEncoderDiagnostic(const EncoderDiagnostic& diagnostic, const AxisConfig& config);
bool printPins(const Axis& axis);
void serviceTelemetry();
