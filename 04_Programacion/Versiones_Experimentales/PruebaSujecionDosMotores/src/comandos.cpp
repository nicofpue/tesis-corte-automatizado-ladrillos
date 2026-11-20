#include "comandos.h"
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
Command Commands::parse(const char* p) {
    Command c;
    while (isspace(static_cast<unsigned char>(*p))) ++p;
    // ON/OFF no contienen S/X: conservar parada inmediata por esos caracteres.
    const char* tail=p;while (*tail&&!isspace(static_cast<unsigned char>(*tail))) ++tail;
    const char* endWord=tail;while (isspace(static_cast<unsigned char>(*tail))) ++tail;
    if (!*tail && endWord-p>=2 && toupper(p[0])=='O') {
        if (endWord-p==2 && toupper(p[1])=='N') { c.type=CommandType::ContactorOn;return c; }
        if (endWord-p==3 && toupper(p[1])=='F' && toupper(p[2])=='F') { c.type=CommandType::ContactorOff;return c; }
    }
    if (!*tail && endWord-p==2 && p[1]=='3') {
        const int prefix=toupper(static_cast<unsigned char>(p[0]));
        if (prefix=='H') { c.type=CommandType::Home3;return c; }
        if (prefix=='E') { c.type=CommandType::End3;return c; }
    }
    // M3,distancia_mm,sentido: distancia positiva; sentido 1 o -1.
    if (toupper(static_cast<unsigned char>(p[0]))=='M' && p[1]=='3') {
        p+=2;
        float values[2];
        for (unsigned i=0;i<2;++i) {
            while (isspace(static_cast<unsigned char>(*p))) ++p;
            if (*p!=',') return c;
            ++p;char* end=nullptr;errno=0;
            values[i]=strtof(p,&end);
            if (end==p||!isfinite(values[i])||errno==ERANGE) return c;
            p=end;
        }
        while (isspace(static_cast<unsigned char>(*p))) ++p;
        if (*p||values[0]<=0||(values[1]!=1&&values[1]!=-1)) return c;
        c.type=CommandType::Move3;c.value=values[0];c.direction=static_cast<int8_t>(values[1]);return c;
    }
    const char kind=toupper(static_cast<unsigned char>(*p));
    if (!kind) return c;
    ++p;
    while (isspace(static_cast<unsigned char>(*p))) ++p;
    if ((kind=='D'||kind=='T')&&*p==',') {
        ++p; char* end=nullptr; errno=0;
        const float v=strtof(p,&end);
        if (end==p||!isfinite(v)||errno==ERANGE) return c;
        while (isspace(static_cast<unsigned char>(*end))) ++end;
        if (*end) return c;
        c.type=kind=='D'?CommandType::Separation:CommandType::Translate;
        c.value=v; return c;
    }
    if (*p) return c;
    switch (kind) {
    case 'D': c.type=CommandType::Separation; c.value=NAN; break;
    case 'H': case 'L': c.type=CommandType::Home; break;
    case 'C': c.type=CommandType::Center; break;
    case 'S': case 'X': c.type=CommandType::Stop; break;
    case 'Q': c.type=CommandType::Status; break;
    case 'E': c.type=CommandType::Encoder; break;
    case '?': c.type=CommandType::Help; break;
    }
    return c;
}
bool Commands::poll(Command& c) {
    for (unsigned n=0;n<32&&Serial.available()>0;++n) {
        const char ch=static_cast<char>(Serial.read());
        // S original y X alias: parar al recibir el caracter, sin esperar Enter.
        if (ch=='S'||ch=='s'||ch=='X'||ch=='x') {
            length_=0; overflow_=false; c=Command{}; c.type=CommandType::Stop; return true;
        }
        if (ch=='\r'||ch=='\n') {
            if (!length_&&!overflow_) continue;
            buffer_[length_]='\0'; c=overflow_?Command{}:parse(buffer_);
            length_=0; overflow_=false; return true;
        }
        if (ch=='\0') { overflow_=true; continue; }
        if (length_<sizeof(buffer_)-1&&!overflow_) buffer_[length_++]=ch;
        else overflow_=true;
    }
    return false;
}
