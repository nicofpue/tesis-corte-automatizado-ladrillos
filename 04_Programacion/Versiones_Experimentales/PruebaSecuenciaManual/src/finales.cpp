#include "finales.h"
void LimitSwitches::begin() {
    pinMode(cfg_.homePin, cfg_.limitInputMode);
    pinMode(cfg_.endPin, cfg_.limitInputMode);
    pinMode(cfg_.contactPin, cfg_.limitInputMode);
}
bool LimitSwitches::home() const { return digitalRead(cfg_.homePin) == cfg_.limitActiveLevel; }
bool LimitSwitches::end() const { return digitalRead(cfg_.endPin) == cfg_.limitActiveLevel; }
bool LimitSwitches::contact() const { return digitalRead(cfg_.contactPin) == cfg_.limitActiveLevel; }
