#pragma once

#include "eeprom.h"

void calibratePwmRange (void);
void calibratePwmServo (const uint32_t servoIdx);

uint16_t getPwmByPos (const uint32_t servoIdx, const float pos);
float getErrorPercentage (const uint32_t servoIdx);
ErrorCond  storePwmByPosToEeprom (void);
ErrorCond  getPwmByPosFromEeprom (void);
