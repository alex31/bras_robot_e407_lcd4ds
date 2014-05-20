#include "lcdDisplay.h"
#include "globalVar.h"
#include "stdutil.h"
#include "ch.h"
#include "printf.h"
#include "rtcAccess.h"
#include "servo_pwm.h"
#include "jbus485.h"
#include "jbusEther.h"
#include "jbus_common.h"
#include "logicLevelIO.h"
#include "analogicIO.h"
#include "IOmode.h"
#include "calibration.h"
#include "cosmosOffsetArray.h"
#include "errorLed.h"
#include <string.h>

#if defined OLED
#include "oledDisplay.h"
#elif defined PICASO4
#include "picaso4Display.h"
#endif

/*
  width  30 char
  height 29
    000000000111111111122222222223
    123456789012345678901234567890
             .      .  .         .
00 |date et heure   .  .         .
01 |
02 |
03 |        
04 |        
05 |        
06 |        
07 |        
08 |        
09 |        
10 |        
11 |        
12 |        
13 |        
14 |        
15 |
16 |        
17 |
18 |                       
19 |                
20 |
21 |                
22 |                         
23 |   
*/

#define clampColor(r,v,b) ((r & 0x1f) <<11 | (v & 0x3f) << 5 | (b & 0x1f))
#define colorDecTo16b(r,v,b) (clampColor((r*31/100), (v*63/100), (b*31/100)))
 
static const char* weekDays[] = {"Dim", "Lun", "Mar", "Mer", "Jeu", "Ven", "Sam"};
static const char pwmStateChar[] = {'I', 'J', 'A', 'T', 'D'};
static const char* pwmStateStr[] = {"IMO", "BLK", "ACC", "MAX", "DEC"};
static const char* servoName[] = {"Toure", "Epaul", "Coude", "Poign", "Pince"};
static LcdDisplayMode lcdDisplayMode =  SPLIT_PAGE;
//static LcdDisplayMode lcdDisplayMode =  COSMOS;
static uint16_t lcd1_val=42;
static uint16_t lcd2_val=42;
static oledConfig olc;

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
#define CHAR_WIDTH 8
#define CHAR_HEIGHT 12
#define NB_CHAR_ON_WIDTH (SCREEN_WIDTH/CHAR_WIDTH)
#define NB_CHAR_ON_HEIGHT ((SCREEN_HEIGHT/CHAR_HEIGHT)-1)


typedef enum {EC_COLOR0, EC_COLOR1, EC_COLOR2, EC_COLOR3, EC_COLOR4, 
	      EC_COLOR5, EC_COLOR6, EC_COLOR7, EC_COLOR8, EC_COLOR_NONE,
	      EC_COL=0200, EC_BLINK, EC_NOBLINK} EscapeCommand;

typedef struct 
{
  int32_t idx;
  uint32_t repeat;
  char buffer [SYSLOG_BUFFLEN][SYSLOG_LINELEN] ;
  char printCopyBuffer[SYSLOG_BUFFLEN][SYSLOG_LINELEN] ;
  Mutex mutex;
} SyslogCirBuf;

typedef enum {TS_NONE, TS_SEC, TS_MIN_SEC} TimeStampFmt;

static SyslogCirBuf syslogCirBuf;
static msg_t bufferQueue[8];
static MAILBOX_DECL(xQueue, bufferQueue, ARRAY_LEN(bufferQueue));


static void displayHour (void);
static void displayPwmInfo (void);
static void displayLogicInfo (void);
static void displayLCDRegister (void);
static void displayAnalogInputInfo (void);
static void displayAnalogSensorsInfo (void);
static void displayJbus485Info (void);
static void displayJbusEtherInfo (void);
static void displaySyslog (uint32_t nb);
static bool_t displayCosmos (void);
static int32_t getSyslogIdx (int32_t historyIdx);
static const char* getSyslog (int32_t historyIdx, TimeStampFmt tsm);
static void OLED_printPL(uint8_t lineNb, uint8_t colorIndex, const char *fmt, ...);
static bool_t isBlinkNoDisplay(void);
static char * bargraph(const float normalised);

void syslogErrorFromISR (const char* msg)
{
  chMBPostI (&xQueue, (msg_t) msg);
}

#define TSLEN (6)
void syslog (LogSeverity ls, const char *fmt, ...)
{
  va_list ap;
  char tmp [SYSLOG_LINELEN], tmp1[SYSLOG_LINELEN];
  static uint32_t repeat=0;

  chMtxLock(&syslogCirBuf.mutex);
  switch (ls) {
  case LOG_INFO : lightFlash (ERROR_GREEN_LED, ls, 2000, 200, 1, 3000); break;
  case LOG_WARN : lightFlash (ERROR_GREEN_LED, ls, 2000, 200, 2, 5000); break;
  case LOG_ERROR : lightFlash (ERROR_GREEN_LED, ls, 2000, 200, 3, 15000); break;
  case LOG_FATAL : lightFlash (ERROR_GREEN_LED, ls, 2000, 200, 4, ERROR_LED_TIME_INFINITE); break;
  }

  /* oledAcquireLock (&olc); */
  /* oledSound (&olc, 1000, 1000); */
  /* oledReleaseLock (&olc); */

  struct Balises {
    uint8_t st[5];
  };
  // See enum EscapeCommand codes
  static const struct Balises bal[4] = {
    {.st = {EC_COL, EC_COLOR5}},
    {.st = {EC_COL, EC_COLOR6}},
    {.st = {EC_COL, EC_COLOR7}},
    {.st = {EC_COL, EC_COLOR7, EC_BLINK}}
  };

  va_start(ap, fmt);
  chvsnprintf (tmp1, SYSLOG_LINELEN-TSLEN, fmt, ap); 
  va_end(ap);
  chsnprintf (tmp, SYSLOG_LINELEN-TSLEN, "%s%s", bal[ls].st, tmp1); 

  if (repeat == 0) {
    if (strncmp (tmp, &(syslogCirBuf.buffer[getSyslogIdx(0)][TSLEN]), SYSLOG_LINELEN-TSLEN) != 0) {
      //      DebugTrace ("'%s' ne '%s'", tmp, &(syslogCirBuf.buffer[getSyslogIdx(0)][TSLEN]));
      chsnprintf (syslogCirBuf.buffer[syslogCirBuf.idx], TSLEN, "%.02d:%.02d>", getMinute(), getSecond()); 
      memcpy (&(syslogCirBuf.buffer[syslogCirBuf.idx][TSLEN]), tmp, SYSLOG_LINELEN-TSLEN);
      syslogCirBuf.idx = (syslogCirBuf.idx+1) % SYSLOG_BUFFLEN;
    } else {
      //      DebugTrace ("repeat first time");
      repeat++;
      chsnprintf (syslogCirBuf.buffer[syslogCirBuf.idx], TSLEN, "%.02d:%.02d>", getMinute(), getSecond());
      chsnprintf (&(syslogCirBuf.buffer[syslogCirBuf.idx][TSLEN]), SYSLOG_LINELEN-TSLEN, " Repete %d fois", repeat+1);
      syslogCirBuf.idx = (syslogCirBuf.idx+1) % SYSLOG_BUFFLEN;
    }
  } else {
    if (strncmp (tmp, &(syslogCirBuf.buffer[getSyslogIdx(1)][TSLEN]), SYSLOG_LINELEN-TSLEN) != 0) {
      //      DebugTrace ("stop repeat");
      chsnprintf (syslogCirBuf.buffer[syslogCirBuf.idx], TSLEN, "%.02d:%.02d>", getMinute(), getSecond()); 
      memcpy (&(syslogCirBuf.buffer[syslogCirBuf.idx][TSLEN]), tmp, SYSLOG_LINELEN-TSLEN);
      syslogCirBuf.idx = (syslogCirBuf.idx+1) % SYSLOG_BUFFLEN;
      repeat = 0;
    } else {
      //      DebugTrace ("more repeat");
      repeat++;
      chsnprintf (syslogCirBuf.buffer[getSyslogIdx(0)], TSLEN, "%.02d:%.02d>", getMinute(), getSecond());
      chsnprintf (&(syslogCirBuf.buffer[getSyslogIdx(0)][TSLEN]), SYSLOG_LINELEN-TSLEN, " Repete %d fois", repeat+1);
    }
  }

  syslogCirBuf.buffer[syslogCirBuf.idx][SYSLOG_LINELEN-1]=0;
  chMtxUnlock();
}

static const char* getSyslog (int32_t historyIdx, TimeStampFmt tsm)
{
  const uint32_t offset = (tsm == TS_NONE) ? TSLEN :  (tsm == TS_SEC) ?  TSLEN/2 : 0;

   return &(syslogCirBuf.buffer[getSyslogIdx(historyIdx)][offset]);
}

static int32_t getSyslogIdx (int32_t historyIdx)
{
  int32_t idx = syslogCirBuf.idx - historyIdx - 1;
  while (idx < 0) idx += SYSLOG_BUFFLEN;
  return idx;
}


bool_t initLcdDisplay (void)
{
  syslogCirBuf.idx = syslogCirBuf.repeat = 0;
  memset (syslogCirBuf.buffer, 0, SYSLOG_BUFFLEN*SYSLOG_LINELEN);
  memset (syslogCirBuf.printCopyBuffer, 0, SYSLOG_BUFFLEN*SYSLOG_LINELEN);
  chMtxInit(&syslogCirBuf.mutex);

  oledInit (&olc, &SD6, 256000, GPIOC, GPIOC_LCD_RST, PICASO);
  //  oledInitSdCard (&olc);

  oledSetTextBgColorTable (&olc, COLOR_BLACK, 0,0,0);
  oledSetTextFgColorTable (&olc, COLOR_BLACK, 100,100,100);
  // Clock color
  oledSetTextBgColorTable (&olc, COLOR_CLOCK , 100,100,100);
  oledSetTextFgColorTable (&olc, COLOR_CLOCK , 0,0,0);
  // Pwm color
  oledSetTextBgColorTable (&olc, COLOR_PWM , 20,40,20);
  oledSetTextFgColorTable (&olc, COLOR_PWM , 30,100,30);
  // input mode color
  oledSetTextBgColorTable (&olc, COLOR_INPUT , 40,20,20);
  oledSetTextFgColorTable (&olc, COLOR_INPUT, 100,50,50);
  // analog sensors color
  oledSetTextBgColorTable (&olc, COLOR_ANALOG , 40,30,0);    
  oledSetTextFgColorTable (&olc, COLOR_ANALOG, 100,70,0);
  
  // Log INFO Color
  oledSetTextBgColorTable (&olc, COLOR_INFO , 0,0,0);
  oledSetTextFgColorTable (&olc, COLOR_INFO , 0,50,100);
  // Log WARNING Color
  oledSetTextBgColorTable (&olc, COLOR_WARNING , 0,0,0);
  oledSetTextFgColorTable (&olc, COLOR_WARNING , 100,100,0);
  // Log ERROR Color
  oledSetTextBgColorTable (&olc, COLOR_ERROR , 0,0,0);
  oledSetTextFgColorTable (&olc, COLOR_ERROR , 100,20,20);

  //  oledPrintVersion (&olc);
  //  chThdSleepMilliseconds(2500);
  oledClearScreen (&olc);

  syslog (LOG_INFO, "start %.02dH%.02d:%.02d", getHour(), getMinute(), getSecond());
  syslog (LOG_WARN, "warn");
  syslog (LOG_ERROR, "error");
  syslog (LOG_FATAL, "fatal");

  return TRUE;
}

msg_t thdLcdDisplay(void *arg) 
{
  (void)arg;
  chRegSetThreadName("thd LCD DISPLAY");
  const char * isrEvent;

  initLcdDisplay ();

  while (!chThdShouldTerminate()) {

    oledAcquireLock (&olc);
    // test if we have some isr syslog queued
    while (chMBFetch (&xQueue, (msg_t *) &isrEvent, TIME_IMMEDIATE) == RDY_OK) {
      syslog (LOG_ERROR, isrEvent);
    }

    if (lcdDisplayMode == SPLIT_PAGE) {
      displayHour ();
      displayPwmInfo ();

      switch (getMode()) {
      case IO_mode_Off:
	displayLCDRegister ();
	break;

      case Mode_Logic:
	displayLogicInfo ();
	break;

      case Mode_Analog_Pos:
	displayAnalogInputInfo ();
	break;

      case Mode_Jbus485:
	displayJbus485Info();
	break;

      case Mode_JbusEther:
	displayJbusEtherInfo();
	break;
	
      case IO_mode_End:
      default:
	break;
      }

      displayAnalogSensorsInfo ();

      displaySyslog(NB_CHAR_ON_HEIGHT-16);
    } else if (lcdDisplayMode == SYSLOG) {
      displaySyslog(NB_CHAR_ON_HEIGHT);
    } else if (lcdDisplayMode == COSMOS) {
      if (displayCosmos() == FALSE) {
	lcdDisplayMode = SPLIT_PAGE;
      }
    }
    
    oledReleaseLock (&olc);
    //    chThdSleepMilliseconds (1);
  }
  return 0;
}



static void OLED_printPL(uint8_t lineNb, uint8_t colorIndex, const char *fmt, ...)
{
  char buffer[NB_CHAR_ON_WIDTH*2];
  char *subBuf = buffer;
  char *nextSubBuf = buffer;
  bool_t moreSubBuf= FALSE;
  uint8_t xpos = 0;
  uint16_t subLen;
  uint8_t savedChar;
  uint8_t curEndIndex;

  va_list ap;
  va_start(ap, fmt);
  chvsnprintf (buffer, NB_CHAR_ON_WIDTH, fmt, ap); 
  va_end(ap);

  // complete with space
  for (uint8_t i=strlen(buffer); i<NB_CHAR_ON_WIDTH; i++) {
    buffer[i]=' ';
  }
  buffer[NB_CHAR_ON_WIDTH] = 0;
  curEndIndex = NB_CHAR_ON_WIDTH;

  do {
    subLen=0;
    char *exploreSubBuf = subBuf = nextSubBuf;
    oledGotoXY (&olc, xpos, lineNb);
    if (colorIndex != EC_COLOR_NONE)
      oledUseColorIndex (&olc, colorIndex);
    
    while ((*exploreSubBuf != 0) && (*exploreSubBuf < EC_COL)) {
      exploreSubBuf++;
      subLen++;
    }

    moreSubBuf = (*exploreSubBuf != 0);
    if (moreSubBuf) {
      const char escapeCode = *exploreSubBuf;
      switch (escapeCode) {
      case EC_COL : colorIndex = *(exploreSubBuf+1); 
	nextSubBuf = exploreSubBuf+2 ;
	buffer[curEndIndex] = buffer[curEndIndex+1] = ' ';
	curEndIndex +=2;
	buffer[curEndIndex] = 0;
	break;
      case EC_BLINK : {
	nextSubBuf = ++exploreSubBuf;
	buffer[curEndIndex++] =  ' ';
	buffer[curEndIndex] = 0;
	if (isBlinkNoDisplay()) {
	  while ((*exploreSubBuf != 0) && (*exploreSubBuf < EC_COL))
	    *exploreSubBuf++ = ' ';
	}
      } ; break;
      case EC_NOBLINK : {
	nextSubBuf = ++exploreSubBuf;
	buffer[curEndIndex++] =  ' ';
	buffer[curEndIndex] = 0;
      } ; break;
      }
    }
    
    if (subLen) {
      if (moreSubBuf) {
	savedChar=subBuf[subLen];
	subBuf[subLen]=0;
      }
      oledPrintBuffer (&olc, subBuf);
      if (moreSubBuf) {
	subBuf[subLen]=savedChar;
      }
      xpos += subLen;
    }

  } while (moreSubBuf) ;
}


static void displayHour (void)
{
  OLED_printPL(0,COLOR_CLOCK, "%s %.02d/%.02d/%.02d %.02d:%.02d:%.02d",
		weekDays[getWeekDay()],
		getMonthDay(), getMonth(), getYear(),
		getHour(), getMinute(), getSecond());
}

#define F2D(x) (lerpu32 (0, 99, x))
//#define PST(idx) (pwmStateChar[servoGetState(idx)])
#define PST(idx) (pwmStateStr[servoGetState(idx)])
#define ESC(idx)  (isServoEngaged(idx) ? EC_NOBLINK : EC_BLINK)
//#define ESC(idx)  (servoGetState(idx) == idx%2 ? EC_BLINK : EC_NOBLINK)


static void displayPwmInfo (void)
{
  uint32_t initPos = 7;
  
  for (uint8_t i=0; i<SERVO_COUNT; i++) {
    OLED_printPL (initPos++, COLOR_PWM, 
		  "%c%d P=%.02d/%.02d:%.02d E=%s V=%.1f/%.1f%c",
		  ESC(i), i, 
		  F2D(servoGetCurrentPos(i)), F2D(servoGetPos(i)), F2D(analogGetRawPos(i)),
		  PST(i), servoGetCurrentSpeed(i), servoGetTopSpeed(i), EC_NOBLINK);
  }
}

#define L2S(idx) (getInputs (idx) & BACKWARD_BIT) ? "BAS   " : "HAUT  ",	\
		 (getInputs (idx) & MOVE_IN_BIT) ? "  ---  " : "*******"
#define L2ST(idx) (getInputs (idx) & BACKWARD_BIT) ? "GAUCHE" : "DROITE",	\
		 (getInputs (idx) & MOVE_IN_BIT) ? "  ---  " : "*******"
static void displayLogicInfo (void)
{
  uint32_t initPos = 1;
  
  OLED_printPL (initPos++, COLOR_INPUT, "  Entrees Logiques");
  OLED_printPL (initPos++, COLOR_INPUT, "Tourelle %s / %s  ", L2ST(0));
  OLED_printPL (initPos++, COLOR_INPUT, "Epaule   %s / %s  ", L2S(1));
  OLED_printPL (initPos++, COLOR_INPUT, "Coude    %s / %s  ", L2S(2));
  OLED_printPL (initPos++, COLOR_INPUT, "Poignet  %s / %s  ", L2S(3));
  OLED_printPL (initPos++, COLOR_INPUT, "Pince    %s / %s  ", L2S(4));
}

static void displayLCDRegister (void)
{
  uint32_t initPos = 1;
  
  OLED_printPL (initPos++, COLOR_INPUT, "  Position  OFF   ");
  OLED_printPL (initPos++, COLOR_INPUT, "  -------------   ");
  OLED_printPL (initPos++, COLOR_INPUT, "LCD1=%d", lcd1_val);
  OLED_printPL (initPos++, COLOR_INPUT, "LCD2=%d", lcd2_val);
  OLED_printPL (initPos++, COLOR_INPUT, "  -------------   ");

}


#define CAN(x) (analogGetCmd(x) * 3.3f)
static void displayAnalogInputInfo (void)
{
  uint32_t initPos = 1;
  OLED_printPL (initPos++, COLOR_INPUT," Entrees Analogiques");
  for (uint8_t i=0; i<SERVO_COUNT; i++) {
    OLED_printPL (initPos++, COLOR_INPUT, "%d %.3f %s", 
		  i,  CAN(i), bargraph(analogGetCmd(i)));
  }

}

static void displayAnalogSensorsInfo (void)
{
  uint32_t initPos = 12;
  for (uint8_t i=0; i<SERVO_COUNT; i++) {
    OLED_printPL (initPos++, COLOR_ANALOG, "%s Err=%.2f I=%.2f P=%.2f", 
		  servoName[i], getErrorRatio(i), analogGetCurrentIntensity(i), 
		  analogGetRawPos(i)*3.3f);
  }
}



static void displayJbus485Info (void)
{
  uint32_t initPos = 1;
  OLED_printPL (initPos++, COLOR_INPUT, "JBUS %s; Addresse: %d", 
		getJbusSerialMode() == MB_RTU ? "RTU" : "ASCII",
		getMbAddressFor485 ());

  OLED_printPL (initPos++, COLOR_INPUT, "%d bauds %d bit(s) de stop",
		getBaudRate(), 
		getNbBitsStop ());

  OLED_printPL (initPos++, COLOR_INPUT, "%s",
		getParity() == MB_PAR_NONE ? "Sans parite" : getParity() == MB_PAR_ODD ? "parite  impaire" : 
		"parite paire");

  OLED_printPL (initPos++, COLOR_INPUT, "%s",  getLastFunctionL1());
  OLED_printPL (initPos++, COLOR_INPUT, "%s",  getLastFunctionL2());
  OLED_printPL (initPos++, COLOR_INPUT, "%s",  getLastFunctionL3());
}


static void displayJbusEtherInfo (void)
{
  uint32_t initPos = 1;
  OLED_printPL (initPos++, COLOR_INPUT, "JBUS IP; addr: %d.%d.%d.%d", 	      
		getIPAddress(0), getIPAddress(1), getIPAddress(2), getIPAddress(3));
  OLED_printPL (initPos++, COLOR_INPUT, "netmask    %d.%d.%d.%d", 
		getNetmask(0), getNetmask(1), getNetmask(2), getNetmask(3));
  OLED_printPL (initPos++, COLOR_INPUT, "ethernet   %x:%x:%x:%x:%x:%x", 
		getMacEther(0),getMacEther(1), getMacEther(2),getMacEther(3),
		getMacEther(4),getMacEther(5));
  
  OLED_printPL (initPos++, COLOR_INPUT, "%s",  getLastFunctionL1());
  OLED_printPL (initPos++, COLOR_INPUT, "%s",  getLastFunctionL2());
  OLED_printPL (initPos++, COLOR_INPUT, "%s",  getLastFunctionL3());
}


static void displaySyslog (uint32_t nb)
{
  char min[3]="   ", sec[3]="   ";

  chMtxLock(&syslogCirBuf.mutex);
  //  for (uint32_t i=0; i<nb; i++) {
  for (int32_t i=nb-1; i>=0; i--) {
    const char* slmsg = getSyslog (i, TS_MIN_SEC);
    if (strncmp (slmsg, min, ARRAY_LEN(min)) != 0) {
      // minutes differs, print entire timestamp
      memcpy (min, slmsg, ARRAY_LEN(min));
      OLED_printPL (NB_CHAR_ON_HEIGHT-i,  EC_COLOR0, "%s", getSyslog (i, TS_MIN_SEC));
    } else {
      // in the same minute, test second
         if (strncmp (&slmsg[ARRAY_LEN(min)], sec, ARRAY_LEN(sec)) != 0) {
	   // seconds differs, print timestamp with only seconds
	   memcpy (sec, &slmsg[ARRAY_LEN(min)], ARRAY_LEN(sec));
	   OLED_printPL (NB_CHAR_ON_HEIGHT-i,  EC_COLOR0, "%s", getSyslog (i, TS_SEC));
	 } else {
	   // in the same minute, same second
	   OLED_printPL (NB_CHAR_ON_HEIGHT-i,  EC_COLOR0, "%s", getSyslog (i, TS_NONE));
	 }
    }
  }
  chMtxUnlock();
}


static bool_t displayCosmos ()
{
  static bool fileError;
  static uint32_t index = 0;
  uint16_t fileHandle;

  if (index == 0) {
    oledListSdCardDirectory (&olc);
    oledSetSoundVolume (&olc, 50);
    oledPlayWav (&olc, "C.WAV");
  }
  
  uint32_t timeStamp = chTimeNow();
  fileError = oledOpenFile (&olc, "C2.GCI", &fileHandle) != 0;
  if (!fileError) {
    oledDisplayGci (&olc, fileHandle, cosmosOffsetArray[index]);
    index = (index+1) % ARRAY_LEN(cosmosOffsetArray);
    oledCloseFile (&olc, fileHandle);
  }
  
  if (chTimeNow() < (timeStamp+1020))
    chThdSleepUntil (timeStamp + 1020);

  return (index != 0);
}


static bool_t isBlinkNoDisplay (void)
{
  return (chTimeNow() / 1000) % 2;
}

void setSysLogMode (LcdDisplayMode mode)
{
  lcdDisplayMode = mode;
}

void setLcd1Register (uint16_t v)
{
  lcd1_val=v;
  syslog (LOG_INFO, "lcd1=%d", v);
}

void setLcd2Register (uint16_t v)
{
  lcd2_val=v;
  syslog (LOG_INFO, "lcd2=%d", v);
}

uint16_t getLcd1Register (void)
{
  return lcd1_val;
}

uint16_t getLcd2Register (void)
{
  return lcd2_val;
}

uint16_t getSyslogDataAsPlcRegister (uint16_t regAddr)
{
  if (regAddr*2 > (SYSLOG_BUFFLEN*SYSLOG_LINELEN)-1 ) {
    return 0;
  } else {
    const char* sysLogBuf = &(syslogCirBuf.printCopyBuffer[0][0]);

    return (sysLogBuf[regAddr*2] << 8) | sysLogBuf[1+(regAddr*2)];
  }
}

void printCopySyslogDataForPlc (void)
{
  char min[3]="   ", sec[3]="   ";
  const uint32_t nb = SYSLOG_BUFFLEN;

  chMtxLock(&syslogCirBuf.mutex);
  //  for (uint32_t i=0; i<nb; i++) {
  for (int32_t i=nb-1; i>=0; i--) {
    const char* slmsg = getSyslog (i, TS_MIN_SEC);
    if (strncmp (slmsg, min, ARRAY_LEN(min)) != 0) {
      // minutes differs, print entire timestamp
      memcpy (min, slmsg, ARRAY_LEN(min));
      chsnprintf (syslogCirBuf.printCopyBuffer[i], SYSLOG_LINELEN,  getSyslog (i, TS_MIN_SEC));
    } else {
      // in the same minute, test second
         if (strncmp (&slmsg[ARRAY_LEN(min)], sec, ARRAY_LEN(sec)) != 0) {
	   // seconds differs, print timestamp with only seconds
	   memcpy (sec, &slmsg[ARRAY_LEN(min)], ARRAY_LEN(sec));
	   chsnprintf (syslogCirBuf.printCopyBuffer[i], SYSLOG_LINELEN,  getSyslog (i, TS_SEC));
	 } else {
	   // in the same minute, same second
	   chsnprintf (syslogCirBuf.printCopyBuffer[i], SYSLOG_LINELEN,  getSyslog (i, TS_NONE));
	 }
    }
  }
  chMtxUnlock();
}




static char * bargraph(const float normalised)
{
  static char bgraph[20];
  const uint32_t len = MIN(sizeof(bgraph)-1, lerpu32 (0, sizeof(bgraph)-1, normalised));
  memset (bgraph, '=', len);
  bgraph[len]=0;
  return bgraph;
}

