#pragma once
#include "eje.h"
// Devuelve false si la cola esta llena; el informe reintenta ese paso.
bool printReply(const char* text);
bool printStatus(const Axis& axis);
bool printEncoder(const Axis& axis);
bool printEncoderDiagnostic(const EncoderDiagnostic& diagnostic, const AxisConfig& config);
bool printPins(const Axis& axis);
void serviceTelemetry();
