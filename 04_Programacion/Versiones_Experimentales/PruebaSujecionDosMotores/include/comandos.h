#pragma once
#include <Arduino.h>
enum class CommandType { ContactorOn, ContactorOff, Home, Separation, Center, Translate, Move3, Home3, End3, Stop, Status, Encoder, Help, Invalid };
struct Command { CommandType type=CommandType::Invalid; float value=0; int8_t direction=1; };
class Commands {
public:
    bool poll(Command& c);
    static Command parse(const char* line);
private:
    char buffer_[64]={}; size_t length_=0; bool overflow_=false;
};
