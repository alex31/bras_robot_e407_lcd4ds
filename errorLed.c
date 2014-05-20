#include "ch.h"
#include "hal.h"

#include "errorLed.h"
#include "stdutil.h"
#include "bitband.h"
#include "printf.h"
#include "globalVar.h"

#define LED_COUNT 3
static WORKING_AREA(waErrorLedThd, 256);

#define PROTECT_DATA_WITH_MUTEX 0

typedef struct {
  uint32_t setTimestamp;
  uint16_t onWidth;
  uint16_t dutyWidth;
  uint16_t duration;
  uint8_t  lastPrio;      
  uint8_t  nbFlash;     
} LedPinParams;

#if PROTECT_DATA_WITH_MUTEX
static Mutex    mutex;
#endif

static LedPinParams ledPinParams[] = {
  {.setTimestamp = 0, .onWidth = 0, .dutyWidth = 0, .duration = 0, .lastPrio=0},
  {.setTimestamp = 0, .onWidth = 0, .dutyWidth = 0, .duration = 0, .lastPrio=0},
  {.setTimestamp = 0, .onWidth = 0, .dutyWidth = 0, .duration = 0, .lastPrio=0}
};


typedef struct {
  GpioPin gpio;
  bool_t  activeLow;
} GpioLed;


static const GpioLed  ledPins[] = {
  {.gpio = {.gpio = GPIOF, .pin = GPIOF_LED_RED},   .activeLow = FALSE}, 
  {.gpio = {.gpio = GPIOG, .pin = GPIOG_LED_GREEN}, .activeLow = FALSE}, 
  {.gpio = {.gpio = GPIOC, .pin = GPIOC_LED},       .activeLow = TRUE}
}; 

#ifndef __COVERITY__
_Static_assert(  ARRAY_LEN(ledPins) == LED_COUNT, 
		"ledPins should be array of same"
		" length than macro LED_COUNT");
_Static_assert(  ARRAY_LEN(ledPinParams) == LED_COUNT, 
		"ledPinParams should be array of same"
		" length than macro LED_COUNT");
#endif

static __attribute__((noreturn)) msg_t errorLedThd(void *arg) ;


void lightOn (const uint32_t ledIndex, const uint8_t errPrio, const uint16_t duration)
{
  lightFlash (ledIndex, errPrio, 1000, 1000, 1, duration); 
}

void lightOff (const uint32_t ledIndex)
{
  if (ledIndex >= LED_COUNT) 
    return;

#if PROTECT_DATA_WITH_MUTEX
  chMtxLock(&mutex);
#endif

  LedPinParams * lpp = &ledPinParams[ledIndex];
  
  lpp->setTimestamp = 0;
  lpp->onWidth = 0;
  lpp->dutyWidth = 0;  
  lpp->duration = 0;   
  lpp->lastPrio = 0;

#if PROTECT_DATA_WITH_MUTEX
  chMtxUnlock ();
#endif
}

void lightFlash (const uint32_t ledIndex, const uint8_t errPrio, const uint16_t dutyWidth,
		 const uint16_t dutyOn, const uint8_t nbFlashConsecutive, const uint16_t duration)
{
  if (ledIndex >= LED_COUNT) 
    return;

  LedPinParams * lpp = &ledPinParams[ledIndex];
  if (errPrio < lpp->lastPrio) 
    return;

#if PROTECT_DATA_WITH_MUTEX
  chMtxLock(&mutex);
#endif
  lpp->setTimestamp = chTimeNow();
  lpp->onWidth = dutyOn;
  lpp->dutyWidth = dutyWidth;  
  lpp->duration = duration;   
  lpp->lastPrio = errPrio;   
  lpp->nbFlash = nbFlashConsecutive;
#if PROTECT_DATA_WITH_MUTEX
  chMtxUnlock ();
#endif
}

void errorLedInit(void)
{
#if PROTECT_DATA_WITH_MUTEX
  chMtxInit(&mutex);
#endif
  chThdCreateStatic(waErrorLedThd, sizeof(waErrorLedThd), NORMALPRIO-1, errorLedThd, 0);  
}


static __attribute__((noreturn)) msg_t errorLedThd(void *arg) 
{
  (void) arg; 
  chRegSetThreadName("thdErrorLed");
  
  while (1) {
#if PROTECT_DATA_WITH_MUTEX
    chMtxLock(&mutex);
#endif
    const uint32_t now = chTimeNow();
    for (uint32_t ledIdx=0; ledIdx<LED_COUNT; ledIdx++) {
      const LedPinParams *lpp = &ledPinParams[ledIdx];
      const  GpioLed *gl = &ledPins[ledIdx];
      const uint32_t durationSinceSet = now - lpp->setTimestamp;
      /* if ((now > 3000) && (ledIdx == ERROR_H407_LED)) { */
      /* 	chprintf (chp, "durationSinceSet=%d lpp->dutyWidth=%d lpp->onWidth=%d\r\n", */
      /* 		  durationSinceSet, lpp->dutyWidth,  lpp->onWidth) ; */
      /* } */
      
      if ((lpp->duration != ERROR_LED_TIME_INFINITE) && (durationSinceSet > lpp->duration)) {
	bb_palWritePad (gl->gpio.gpio, gl->gpio.pin, gl->activeLow);
      } else {
	if (lpp->nbFlash == 0) {
	  bb_palWritePad (gl->gpio.gpio, gl->gpio.pin, gl->activeLow);
	} else {
	  const uint16_t allPulseDuration = (((lpp->nbFlash - 1) * 2) +1) * lpp->onWidth;
	  const uint16_t durationSinceFirstPulse = durationSinceSet % lpp->dutyWidth;

	  if (durationSinceFirstPulse > allPulseDuration) {
	    bb_palWritePad (gl->gpio.gpio, gl->gpio.pin, gl->activeLow);
	  } else {
	    //bb_palWritePad (gl->gpio.gpio, gl->gpio.pin, !gl->activeLow);
	    bb_palWritePad (gl->gpio.gpio, gl->gpio.pin, 
			    gl->activeLow ^ (!((durationSinceFirstPulse / lpp->onWidth) % 2)));
	  }
	}
      }
    }
#if PROTECT_DATA_WITH_MUTEX
  chMtxUnlock ();
#endif
    chThdSleepMilliseconds (10);
  }
}
