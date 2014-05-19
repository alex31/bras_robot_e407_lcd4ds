#pragma once

#include "eeprom.h"

#define CALIBRATION_TABLE_SIZE	  512

void calibratePwmRange (void);
void calibratePwmServo (const uint32_t servoIdx);

uint16_t   getPwmByReadPos (const uint32_t servoIdx, const float pos);
float      getOrderByReadPos (const uint32_t servoIdx, const float pos);
float      getReadPosNormalised (const uint32_t servoIdx);
float      getErrorRatio (const uint32_t servoIdx);
bool_t     isCalibrated(void);
ErrorCond  storePwmByPosToEeprom (void);
ErrorCond  getPwmByPosFromEeprom (void);
