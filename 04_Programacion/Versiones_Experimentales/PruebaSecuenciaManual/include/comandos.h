#pragma once
#include <Arduino.h>
enum class CommandType { Start, Next, Stop, Status, Encoder, Help, Invalid };
struct Command { CommandType type=CommandType::Invalid; };
class Commands {
public:
    bool poll(Command& c);
    static Command parse(const char* line);
private:
    char buffer_[64]={}; size_t length_=0; bool overflow_=false;
};
