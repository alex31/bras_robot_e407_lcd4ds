#ifndef __LCD_DISPLAY_H__
#define __LCD_DISPLAY_H__

#include "ch.h"

#define SYSLOG_BUFFLEN 30
#define SYSLOG_LINELEN 30

typedef enum  {
  SPLIT_PAGE, SYSLOG, COSMOS
} LcdDisplayMode;

typedef enum  {
  LOG_INFO=0, LOG_WARN, LOG_ERROR, LOG_FATAL
} LogSeverity;

typedef enum  {
  COLOR_BLACK=0, COLOR_CLOCK, COLOR_PWM, COLOR_INPUT, COLOR_ANALOG, 
  COLOR_INFO, COLOR_WARNING, COLOR_ERROR 
} GuiColors;

bool_t initLcdDisplay (void);
msg_t thdLcdDisplay(void *arg) ;
void setSysLogMode (LcdDisplayMode mode);

void syslog (LogSeverity ls, const char *fmt, ...);
void syslogErrorFromISR (const char* msg);

void setLcd1Register (uint16_t v);
void setLcd2Register (uint16_t v);
uint16_t getLcd1Register (void);
uint16_t getLcd2Register (void);

void printCopySyslogDataForPlc (void);
uint16_t getSyslogDataAsPlcRegister (uint16_t regAddr);

#endif // __LCD_DISPLAY_H__
