#pragma once
#include <Arduino.h>


void caracterizacionBegin();
void caracterizacionUpdate();
bool caracterizacionBusy();
void cancelTest();
void noteRejectedCommand();
void runStep(float pwm, unsigned long testTime, int dir);
void runCalibration(float pwm, long targetCounts, int dir);
void runApproach(float pwm, unsigned long ms, int dir);
void runRelease(float pwm, unsigned long ms);
