#pragma once


enum ErrorLedIndex {ERROR_LED_RED=0, ERROR_GREEN_LED, ERROR_H407_LED};
#define ERROR_LED_TIME_INFINITE UINT16_MAX

void errorLedInit(void);
void lightOn (const uint32_t ledIndex, const uint8_t errPrio, const uint16_t duration);
void lightFlash (const uint32_t ledIndex, const uint8_t errPrio, const uint16_t dutyWidth,
		 const uint16_t dutyOn, const uint8_t nbFlashConsecutive, const uint16_t duration);
void lightOff (const uint32_t ledIndex);
