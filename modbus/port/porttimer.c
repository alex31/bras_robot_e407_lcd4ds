/*
 * FreeModbus Libary: Atmel AT91SAM3S Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: porttimer.c,v 1.1 2010/06/06 13:07:20 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <ch.h>
#include <hal.h>
#include "stdutil.h"
#include "IOmode.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "modbus/port/port.h"
#include "modbus/include/mb.h"
#include "modbus/include/mbport.h"

/* ----------------------- Defines ------------------------------------------*/



#ifdef DEBUG_MB
#include "tm.h"
static TimeMeasurement tm;
#endif

static void timerHandler(GPTDriver *gptp);

/* ----------------------- Static variables ---------------------------------*/
/*
 * GPT2 configuration.
 */
static const GPTConfig gptcfg = {
  100000,    /* 100kHz timer clock.*/
  timerHandler,    /* Timer callback.*/
  0
};

static systime_t    timerout= 0;

/* ----------------------- Start implementation -----------------------------*/

static void timerHandler(GPTDriver *gptp)
{
  (void)gptp;

#ifdef DEBUG_MB
    tmStopMeasurement (&tm);
    palSetPad (BOARD_LED2_P, BOARD_LED2);
#endif
    
    chSysLockFromIsr(); {
    vMBPortSetWithinException (TRUE) ;
    if (pxMBPortCBTimerExpired () == TRUE)
      rescheduleJbus485FromIsr();
    vMBPortSetWithinException (FALSE) ;
  } chSysUnlockFromIsr();
}


BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
  timerout = usTim1Timerout50us*((500*1000)/gptcfg.frequency);
  gptStart(&GPTDRIVER, &gptcfg);
#ifdef DEBUG_MB
  tmObjectInit (&tm);
#endif
  return TRUE;
}

void
vMBPortTimerClose( void )
{
   gptStop (&GPTDRIVER);
}

void
vMBPortTimersEnable(  )
{
#ifdef DEBUG_MB
  palClearPad (BOARD_LED2_P, BOARD_LED2);
  tmStartMeasurement (&tm);
#endif
  if (bMBPortIsWithinException() == TRUE) {
    gptStopTimerI (&GPTDRIVER);
    gptStartOneShotI(&GPTDRIVER, timerout);
  } else { 
    gptStopTimer (&GPTDRIVER);
    gptStartOneShot(&GPTDRIVER, timerout);
  }
}

void
vMBPortTimersDisable(  )
{
  if (bMBPortIsWithinException() == TRUE)
    gptStopTimerI (&GPTDRIVER);
  else
    gptStopTimer (&GPTDRIVER);
}

void
vMBPortTimersDelay( USHORT usTimeOutMS )
{
  chThdSleepMicroseconds (usTimeOutMS);
}

#ifdef DEBUG_MB
float fMBPortTimerMesurment ()
{
    static const float freqDiv1000 =  (float) halGetCounterFrequency() / 1000.0f;

    return (((float) tm.last) / freqDiv1000);
}
#endif
