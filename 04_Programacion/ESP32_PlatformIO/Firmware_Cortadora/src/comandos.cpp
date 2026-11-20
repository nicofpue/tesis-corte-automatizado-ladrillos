// =====================================================================
// comandos.cpp - Lectura de ordenes por el monitor serie
// ---------------------------------------------------------------------
// =====================================================================
#include "comandos.h"
#include <ctype.h>

// Convierte una linea en orden. Acepta una sola letra (mayuscula o minuscula)
// con espacios alrededor; cualquier otra cosa se devuelve como Invalid.
Command Commands::parse(const char* line) {
    while (isspace(static_cast<unsigned char>(*line))) ++line;
    if (!*line) return {};
    const int key = toupper(static_cast<unsigned char>(*line++));
    while (isspace(static_cast<unsigned char>(*line))) ++line;
    if (*line) return {};
    switch (key) {
    case 'I': return {CommandType::Start};
    case 'S': case 'X': return {CommandType::Stop};
    case 'Q': return {CommandType::Status};
    case 'E': return {CommandType::Encoder};
    case '?': return {CommandType::Help};
    default: return {};
    }
}

// Se llama en cada vuelta de loop() y nunca espera: lee como maximo 32 caracteres.
// STOP (S/X) se devuelve de inmediato por seguridad; las demas ordenes al recibir Enter.
// Una linea mas larga que el buffer se descarta completa.
bool Commands::poll(Command& command) {
    for (unsigned n=0; n<32 && Serial.available()>0; ++n) {
        const char ch = static_cast<char>(Serial.read());
        if (ch=='S' || ch=='s' || ch=='X' || ch=='x') {
            length_=0; overflow_=false; command={CommandType::Stop}; return true;
        }
        if (ch=='\r' || ch=='\n') {
            if (!length_ && !overflow_) continue;
            buffer_[length_]='\0';
            command=overflow_ ? Command{} : parse(buffer_);
            length_=0; overflow_=false; return true;
        }
        if (ch=='\0') { overflow_=true; continue; }
        if (length_<sizeof(buffer_)-1 && !overflow_) buffer_[length_++]=ch;
        else overflow_=true;
    }
    return false;
}
