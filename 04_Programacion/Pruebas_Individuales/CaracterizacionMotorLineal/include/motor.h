#pragma once
#include <Arduino.h>


bool motorBegin();
void setPWM(float porcentaje);
void setDirection(int dir);
void stopMotor();
float getMotorPWM();
uint16_t getMotorDuty();
int getMotorDirection();
bool motorReady();