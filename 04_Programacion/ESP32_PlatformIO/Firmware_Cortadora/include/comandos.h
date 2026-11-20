#pragma once
// =====================================================================
// comandos.h - Ordenes por el monitor serie
// ---------------------------------------------------------------------
// Lee caracteres del USB sin bloquear y los convierte en ordenes:
//   I = START, S o X = STOP, Q = estado, E = diagnostico, ? = ayuda.
// =====================================================================
#include <Arduino.h>
// Tipos de orden que entiende el firmware.
enum class CommandType { Start, Stop, Status, Encoder, Help, Invalid };
struct Command { CommandType type=CommandType::Invalid; };
// Acumula caracteres hasta Enter. S/X se atienden al instante, sin esperar Enter.
class Commands {
public:
    bool poll(Command& c);
    static Command parse(const char* line);
private:
    char buffer_[64]={}; size_t length_=0; bool overflow_=false;
};
