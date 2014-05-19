/*
  ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
  2011,2012 Giovanni Di Sirio.

  This file is part of ChibiOS/RT.

  ChibiOS/RT is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  ChibiOS/RT is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "ch.h"
#include "hal.h"
#include "stdutil.h"
#include "globalVar.h"
#include "servo_pwm.h"
#include "lcdDisplay.h"
#include "IOmode.h"
#include "calibration.h"
#include "printf.h"
#include "bitband.h"
#include <string.h>
#include <math.h>


const float max_current_intensity_calib[SERVO_COUNT] = {
  0.100f, // turret
  0.100f, // elbow
  0.080f, // shoulder
  0.050f, // wrist
  0.040f  // clamp
};

const float max_current_intensity_use[SERVO_COUNT] = {
  0.200f, // turret
  0.200f, // elbow
  0.150f, // shoulder
  0.100f, // wrist
  0.080f  // clamp
};

const uint32_t USE_CURRENT_PWM = 0;

#pragma message \
  "change ACCEL_RATIO in " __FILE__	" from 0.3 to 1.0"\
  " after testing that all changes are OK"

#define ACCEL_RATIO (0.3f) /* acceleration / deceleration range in ratio of full range */
//#define ACCEL_RATIO (1.0f) /* acceleration / deceleration range in ratio of full range */

struct  PwmServo_ {
  PWMDriver	*driver;
  uint32_t	channel;
  GPIO_TypeDef	*gpio;
  uint32_t	pin;
} ;

_Static_assert(sizeof(int8_t) == sizeof (bool_t), "(sizeof(int8_t) != sizeof (bool_t)");
_Static_assert(sizeof(int8_t) == sizeof (AccelDecelState), "(sizeof(int8_t) != sizeof (AccelDecelState)");
_Static_assert((sizeof(Mutex) % 4)==0, "sizeof (Mutex) not 32 bits aligned");



typedef struct {
  // champs à sauver dans l'eeprom
  float		currentPos ; // normalised value
  float		orderPos ;   // normalised value
  float		parkPos; // normalised value -> start position value
  float		topSpeed ;	    // time in second from stop to stop
  uint32_t	pwmMin ;     // pwm value for begin position of servo
  uint32_t	pwmMax ;     // pwm value for end position of servo
  
  // champs à ne pas sauver si restaurer : garder le champs
  // mutex comme premier element de cette partie 
  // l'arithmetique sur les pointeur utilise l'offset entre le
  // debut de la struct et ce champ.
  Mutex         mutex ;      // to avoid race condition in struct access
  const PwmServo *pwmServo ; // ptr to PwmServo struct;
  float		accMax; // cache acceleration for movement calculus
  float		currentSpeed; // calculated speed, should me measured actually
  uint32_t	lastPwm ; // last pwm sent to avoid to send same values to the hardware
  AccelDecelState cachedState; // store the state in the accel/maintain/decel phases
  bool_t	engaged;	// servo commanded or not
} ServoState;

#if defined BOARD_STM32_E407_STX
// channel number begin here @0, but in st doc, it begins @1
const PwmServo servos[] = {
  {.driver = &PWMD3, .channel = 1, .gpio = GPIOB, .pin = GPIOB_PWM_TURRET}, // PIN 5
  {.driver = &PWMD4, .channel = 1, .gpio = GPIOD, .pin = GPIOD_PWM_SHOULDER},// PIN 13
  {.driver = &PWMD4, .channel = 0, .gpio = GPIOD, .pin = GPIOD_PWM_ELBOW}, // PIN 12
  {.driver = &PWMD4, .channel = 3, .gpio = GPIOD, .pin = GPIOD_PWM_WRIST}, // PIN 15
  {.driver = &PWMD4, .channel = 2, .gpio = GPIOD, .pin = GPIOD_PWM_CLAMP} // PIN 14
};

/* const PwmServo servos[] = { */
/*   {.driver = &PWMD4, .channel = 0, .gpio = GPIOD, .pin = GPIOD_PWM_TURRET}, // PIN 12 */
/*   {.driver = &PWMD4, .channel = 1, .gpio = GPIOD, .pin = GPIOD_PWM_SHOULDER}, // PIN 13 */
/*   {.driver = &PWMD4, .channel = 2, .gpio = GPIOD, .pin = GPIOD_PWM_ELBOW}, // PIN 14 */
/*   {.driver = &PWMD4, .channel = 3, .gpio = GPIOD, .pin = GPIOD_PWM_WRIST}, // PIN 15 */
/*   {.driver = &PWMD3, .channel = 1, .gpio = GPIOB, .pin = GPIOB_PWM_CLAMP} // PIN 5 */
/* }; */
#else
#error "one board has to be defined"
#endif



ServoState __attribute__((aligned(8)))
servoStates[] = {
  {.mutex =  _MUTEX_DATA(servoStates[0].mutex), .pwmServo=&servos[0]},
  {.mutex =  _MUTEX_DATA(servoStates[1].mutex), .pwmServo=&servos[1]},
  {.mutex =  _MUTEX_DATA(servoStates[2].mutex), .pwmServo=&servos[2]},
  {.mutex =  _MUTEX_DATA(servoStates[3].mutex), .pwmServo=&servos[3]},
  {.mutex =  _MUTEX_DATA(servoStates[4].mutex), .pwmServo=&servos[4]},
};

_Static_assert( ARRAY_LEN(servos) == ARRAY_LEN(servoStates),
	       "servos and servoStates should be array of same length");

_Static_assert(  ARRAY_LEN(servos) == SERVO_COUNT, 
		"servos and servoStates should be array of same"
		" length than macro SERVO_COUNT defined in servo_pwm.h");

static void servoSetCurrentPos (const uint32_t servoIdx, float pos);
static float getAccelToOrder (const ServoState *st);
static float getMaxAccel ( const ServoState *st, const float dt, const float dist);
static void initAccel (ServoState *st);



void setPwmValueByIndex (const uint32_t servoIdx, const uint32_t value)
{
  if (servoIdx >= SERVO_COUNT) 
    return;

  ServoState *st = &servoStates[servoIdx];
  const PwmServo *srv = &servos[servoIdx];
  setPwmValue(srv, value);
  st->lastPwm = value;
}

void setPwmValue (const PwmServo *s, const uint32_t value)
{
  pwmEnableChannel(s->driver, s->channel, PWM_FRACTION_TO_WIDTH(s->driver, 20000, value)); 
}

#if defined BOARD_STM32_E407_STX
void timerAndPwmInit (void) 
{
  static PWMConfig pwmcfgTim3 = {
    1000*1000,                              /* 1MHz PWM clock frequency.   */
    20000,                                  /* Initial PWM period 20ms.   */
    NULL,                                   /* no callback on PWM counter reset */
    {
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_ACTIVE_HIGH, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL}
    },
    0,
    0
  };
  
  static PWMConfig pwmcfgTim4 = {
    1000*1000,                              /* 1MHz PWM clock frequency.   */
    20000,                                  /* Initial PWM period 20ms.   */
    NULL,                                   /* no callback on PWM counter reset */
    {
      {PWM_OUTPUT_ACTIVE_HIGH, NULL},
      {PWM_OUTPUT_ACTIVE_HIGH, NULL},
      {PWM_OUTPUT_ACTIVE_HIGH, NULL},
      {PWM_OUTPUT_ACTIVE_HIGH, NULL}
    },
    0,
    0
  };
  

  pwmStart(&PWMD3, &pwmcfgTim3);
  pwmStart(&PWMD4, &pwmcfgTim4);

  setPwmValue (&servos[0], 0);
  setPwmValue (&servos[1], 0);
  setPwmValue (&servos[2], 0);
  setPwmValue (&servos[3], 0); 
  setPwmValue (&servos[4], 0); 

  for (uint32_t i=0; i<SERVO_COUNT; i++) {  
    palSetPadMode (servos[i].gpio, servos[i].pin, PAL_MODE_ALTERNATE(2));
  }
}

#else 
#error "one board has to be defined"
#endif


void servoInitAllCurrentPosAndGotoPark (const float backupCurPosIfNotCalibrated[SERVO_COUNT])
{
    for (uint32_t servoIdx=0; servoIdx<SERVO_COUNT; servoIdx++) {  
      if (isCalibrated()) {
	servoSetCurrentPos (servoIdx, getReadPosNormalised (servoIdx));
      } else if (backupCurPosIfNotCalibrated != NULL) {
	servoSetCurrentPos (servoIdx, backupCurPosIfNotCalibrated[servoIdx]);
      } else {
	servoSetCurrentPos (servoIdx, servoGetParkPos(servoIdx));
      }
      servoSetPos (servoIdx, CLAMP_TO(0.0f, 1.0f, servoStates[servoIdx].parkPos));
    }
}

void initAllServos (void)
{
  timerAndPwmInit();

  for (uint32_t i = 0; i < SERVO_COUNT; i++) {
    servoInit (i);
  }

  getServoStatesFromEeprom ();
  
  for (uint32_t i = 0; i < SERVO_COUNT; i++) {
    initAccel (&servoStates[i]);
  }
  
  servoInitAllCurrentPosAndGotoPark (NULL);

}

void servoInit (const uint32_t servoIdx)
{
  ServoState *st = &servoStates[servoIdx];

  chMtxInit(&st->mutex);
  st->currentPos = 0.0f;
  st->parkPos = 0.5f;
  st->topSpeed = 1/3.0f;
  st->currentSpeed = 0.0f;
  st->pwmMin = 1300;    // to protect servo at first flash
  st->pwmMax = 1700;    // real pwmmin/max should be found and saved before real use
  st->lastPwm = 1;    // not set
  st->engaged = TRUE; // servo driven @ power on, if something goes wrong, security will play
  initAccel (st);
}

void servoSetCurrentPosAsPark (const uint32_t servoIdx)
{
  if (servoIdx >= SERVO_COUNT) 
    return;

  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  st->parkPos = CLAMP_TO(0.0f, 1.0f, st->orderPos);
  chMtxUnlock();
}

void servoSetPosI (ServoState *st, float order)
{
  st->orderPos =  CLAMP_TO(0.0f, 1.0f, order);
}

static void servoSetCurrentPos (const uint32_t servoIdx, float pos)
{
  if (servoIdx >= SERVO_COUNT) 
    return;

  ServoState *st = &servoStates[servoIdx];

  if (pos ==  st->currentPos) 
    return;

  chMtxLock(&st->mutex);
  st->currentPos = pos;
  chMtxUnlock();
}

void servoSetPos (const uint32_t servoIdx, float order)
{
  if (servoIdx >= SERVO_COUNT) 
    return;

  ServoState *st = &servoStates[servoIdx];

  if (order ==  st->currentPos) 
    return;

  chMtxLock(&st->mutex);
  servoSetPosI (st, order);
  chMtxUnlock();
}


void servoPark (const uint32_t servoIdx)
{ 
  ServoState *st = &servoStates[servoIdx];
  servoSetPos (servoIdx, st->parkPos);
}

void servoSlidePosTowardEnd (const uint32_t servoIdx, float timeInSecond)
{
  ServoState *st = &servoStates[servoIdx];
  const float increment = timeInSecond * st->topSpeed;
  chprintf (chp, "%d to END topspeed=%.2f\n\r", servoIdx, st->topSpeed);
  servoSetPos (servoIdx, st->orderPos + increment);
}

void servoSlidePosTowardBegin (const uint32_t servoIdx, float timeInSecond)
{
  ServoState *st = &servoStates[servoIdx];
  const float increment = timeInSecond * st->topSpeed;
  chprintf (chp, "%d to BEGIN topspeed=%.2f\n\r", servoIdx, st->topSpeed);

  servoSetPos (servoIdx, st->orderPos - increment);
}


void servoSetTopSpeed (const uint32_t servoIdx, float topSpeed)
{
  if ((servoIdx >= SERVO_COUNT) || (topSpeed == 0))
    return;
  ServoState *st = &servoStates[servoIdx];


  chMtxLock(&st->mutex);
  st->topSpeed = 1.0f / topSpeed;
  initAccel (st);
  chMtxUnlock();
}


void servoSetPwmMin (const uint32_t servoIdx, uint32_t pwmVal)
{
  if (servoIdx >= SERVO_COUNT) 
    return;
  if (pwmVal > 4000) 
    return;

  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  if (pwmVal != USE_CURRENT_PWM) {
    st->pwmMin = pwmVal;
  } else {
    st->pwmMin = lerpu32 (st->pwmMin, st->pwmMax, st->currentPos);
  }
  servoSetPosI (st, 0.0f);
  chMtxUnlock();
}


void servoSetPwmMax (const uint32_t servoIdx, uint32_t pwmVal)
{
  if (servoIdx >= SERVO_COUNT) 
    return;
  if (pwmVal > 4000) 
    return;

  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  if (pwmVal != USE_CURRENT_PWM) {
    st->pwmMax = pwmVal;
  } else {
    st->pwmMax = lerpu32 (st->pwmMin, st->pwmMax, st->currentPos);
  }
  servoSetPosI (st, 1.0f);
  chMtxUnlock();
}


void inversePwmMinAndMax  (const uint32_t servoIdx)
{
   if (servoIdx >= SERVO_COUNT) 
    return;
 
  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  const uint32_t oldMax = st->pwmMax;
  st->pwmMax = st->pwmMin;
  st->pwmMin = oldMax;
  chMtxUnlock();
}

float servoGetPos (const uint32_t servoIdx)
{
  if (servoIdx >= SERVO_COUNT) 
    return -1.0f;
  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  const float val = st->orderPos;
  chMtxUnlock();
  return val;
}

float servoGetParkPos (const uint32_t servoIdx)
{
  if (servoIdx >= SERVO_COUNT) 
    return -1.0f;
  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  const float val = st->parkPos;
  chMtxUnlock();
  return val;
}

float servoGetCurrentPos (const uint32_t servoIdx)
{
  if (servoIdx >= SERVO_COUNT) 
    return -1.0f;
  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  const float val = st->currentPos;
  chMtxUnlock();
  return val;
}


float servoGetTopSpeed (const uint32_t servoIdx)
{
  if (servoIdx >= SERVO_COUNT) 
    return  -1.0f;
  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  const float val = 1.0f/st->topSpeed;
  chMtxUnlock();
  return val;
}

float servoGetCurrentSpeed (const uint32_t servoIdx)
{
  if (servoIdx >= SERVO_COUNT) 
    return  -1.0f;
  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  const float val = st->currentSpeed != 0.0f ? 1.0f / st->currentSpeed : -1.0f;
  chMtxUnlock();
  return val;
}


AccelDecelState servoGetState (const uint32_t servoIdx)
{
  if (servoIdx >= SERVO_COUNT) 
     return ADS_Idle;
  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  AccelDecelState val = st->cachedState;
  chMtxUnlock();
  return val;
}

bool_t isServoMoving (const uint32_t servoIdx)
{
  if (servoIdx >= SERVO_COUNT) 
    return FALSE;
  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
   const bool_t val = (st->currentSpeed != 0.0f) || (st->currentPos != st->orderPos);
  chMtxUnlock();
  return val;
}

bool_t isAtLowStopPos (const uint32_t servoIdx)
{
  if (servoIdx >= SERVO_COUNT) 
    return FALSE;
  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  const bool_t val = (st->currentPos < 0.01f);
  chMtxUnlock();
  return val;
}

bool_t isAtHighStopPos (const uint32_t servoIdx)
{
  if (servoIdx >= SERVO_COUNT) 
    return FALSE;
  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  const bool_t val = (st->currentPos > 0.99f);
  chMtxUnlock();
  return val;
}


uint32_t servoGetPwmMin (const uint32_t servoIdx)
{
  if (servoIdx >= SERVO_COUNT) 
    return 0xffff;
  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  const uint32_t val = st->pwmMin;
  chMtxUnlock();
  return val;
}


uint32_t servoGetPwmMax (const uint32_t servoIdx)
{
  if (servoIdx >= SERVO_COUNT) 
    return 0xffff;
  ServoState *st = &servoStates[servoIdx];

  chMtxLock(&st->mutex);
  const uint32_t val = st->pwmMax;
  chMtxUnlock();
  return val;
}


void servoPeriodicMove (const uint32_t servoIdx, float timeInSecond)
{
  /*
    si on est en pause on fait rien
    sinon
    °on calcule en valeur normalisée la valeur courante correspondant
     au temp passé (passé en argument)
    °on calcule le pwm correspondant
    °on met à jour la sortie pwm
   */
  if (servoIdx >= SERVO_COUNT) 
    return;

  // not to interfere when we calibrate
  if ((getMode() == Mode_Tuning) || (getMode() == IO_mode_Off))
    return;

  ServoState *st = &servoStates[servoIdx];
  chMtxLock(&st->mutex);
  float dist = st->orderPos - st->currentPos;
  const float neededDeacc = getAccelToOrder (st);
  const bool_t goReverse= signbit(dist) != signbit (st->currentSpeed);

#ifdef TRACE_PWM
  DebugTrace ("DBG> order=%.2f curr=%.2f dist=%.3f neededDeacc=%.3f goReverse=%d\n",
    st->orderPos, st->currentPos, dist, neededDeacc, goReverse);
#endif

// nearly stopped
  if ( (fabsf (st->currentSpeed) < 0.1f) && 
    (fabsf (dist) < 0.005f)) {
  goto  shouldStop;
}
      

  if (goReverse || (fabsf (neededDeacc) < st->accMax)) {
    // should accelarate
    const float calcMaxAcc = getMaxAccel (st, timeInSecond, dist);
    const float accel = fabsf (calcMaxAcc) < st->accMax ? calcMaxAcc :
       copysign (st->accMax, dist);
    //  const float newSpeed = st->currentSpeed + (accel * timeInSecond); 
    const float newSpeed = fmaf (accel, timeInSecond, st->currentSpeed);
#ifdef TRACE_PWM
    DebugTrace ("DBG> ACC=%.2f newSpeed=%.2f\n",accel, newSpeed); 
#endif
     
    if (fabsf (newSpeed) <= st->topSpeed) {
      st->currentSpeed = newSpeed;
      st->cachedState = ADS_Accelerate;
    } else {
      st->currentSpeed =  copysign (st->topSpeed, newSpeed) ;
      st->cachedState = ADS_TopSpeed;
#ifdef TRACE_PWM
      DebugTrace ("ADS_TopSpeed");
#endif
    }
  } else {
    // should decelerate
    //const float newSpeed = st->currentSpeed + (neededDeacc * timeInSecond);
    const float newSpeed = fmaf (neededDeacc, timeInSecond, st->currentSpeed);
#ifdef TRACE_PWM
    DebugTrace ("DBG> DEacc=%.2f newSpeed=%.2f\n",neededDeacc, newSpeed); 
#endif
 
    if (signbit (newSpeed) != signbit (st->currentSpeed))
      goto shouldStop;

    st->currentSpeed = newSpeed;
    st->cachedState = ADS_Decelerate;
  }


  const float increment = timeInSecond * st->currentSpeed;
  st->currentPos =   CLAMP_TO (-0.0f, 1.0f, st->currentPos + increment); 
  dist = st->orderPos - st->currentPos;

  // nearly stopped
  if ( (fabsf (st->currentSpeed) < 0.1f) && 
       (fabsf (dist) < 0.005f)) {
  shouldStop:

    st->cachedState = ADS_Idle;
    st->currentSpeed = 0.f;
    st->currentPos = st->orderPos;
  }

  // if disangaged, zero width pulse will render servo inoperative
  const uint32_t curPwm =  st->engaged ? lerpu32 (st->pwmMin, st->pwmMax, st->currentPos) : 0;
  if (curPwm != st->lastPwm) {
    st->lastPwm = curPwm;
    setPwmValue (st->pwmServo, st->engaged ? curPwm : 0);
#ifdef TRACE_PWM
    DebugTrace ("move[%d] : pwm=%d", servoIdx, curPwm);
#endif
  }
  
  chMtxUnlock();
}

void servoEngage (const uint32_t servoIdx)
{
  if (servoIdx == SERVO_ALL_SERVOS) {
    for (uint32_t i=0; i<SERVO_COUNT; i++) {
      servoEngage (i);
    }
  } else if (servoIdx < SERVO_COUNT) {
    ServoState *st = &servoStates[servoIdx];
    if (st->engaged != TRUE) {
      if (isCalibrated()) {
	servoSetCurrentPos (servoIdx, getReadPosNormalised (servoIdx));
      }
      st->engaged = TRUE;
    }
  }
}

void servoDisengage (const uint32_t servoIdx)
{
  if (servoIdx == SERVO_ALL_SERVOS) {
    for (uint32_t i=0; i<SERVO_COUNT; i++) {
      servoDisengage (i);
    }
  } else if (servoIdx < SERVO_COUNT) {
    ServoState *st = &servoStates[servoIdx];
    st->engaged = FALSE;
    setPwmValue (st->pwmServo, 0); // force in case of pwm thread not running
  }
}

bool_t isServoEngaged (const uint32_t servoIdx)
{
  bool_t engaged = TRUE;
  if (servoIdx == SERVO_ALL_SERVOS) {
    for (uint32_t i=0; i<SERVO_COUNT; i++) {
       engaged |= isServoEngaged(i);
    }
  } else if (servoIdx < SERVO_COUNT) {
    const ServoState *st = &servoStates[servoIdx];
    engaged = st->engaged;
  }

  return engaged;
}

static float getAccelToOrder ( const ServoState *st)
{
  if (st->orderPos == st->currentPos)
    return 0.f;

  const float absMaxAcc = ((st->topSpeed * st->topSpeed) / ACCEL_RATIO) *2;
  /*
    in mathomatic :
    orderPos= ((acceleration/2)*t^2) + (currentSpeed*t) + currentPos
    currentSpeed = -acceleration*t
    t -> currentSpeed/acceleration
    replace t with -currentSpeed/acceleration
    -> acceleration = ((currentSpeed*currentSpeed)/(2.0*(currentPos - orderPos)));
   */
  
  const float rawAcc = (st->currentSpeed*st->currentSpeed) /
			     (2.0f*(st->currentPos - st->orderPos));
  return (rawAcc < 0) ? MAX (-absMaxAcc, rawAcc) :  MIN (absMaxAcc, rawAcc);
}

static float getMaxAccel ( const ServoState *st, const float dt, const float dist)
{
  // a = ((-1.0)*(v - nv)/dt);
  const float nv = dist/(dt*2.f);
  const float acc = (nv - st->currentSpeed)/dt;

  return acc;
}

static void initAccel (ServoState *st)
{
  st->accMax = (st->topSpeed * st->topSpeed) / ACCEL_RATIO ;
}

uint32_t getPwmValueByIndex (const uint32_t servoIdx)
{
   if (servoIdx >= SERVO_COUNT) 
    return 0;
  ServoState *st = &servoStates[servoIdx];
  return st->lastPwm;
}


ErrorCond storeServoStatesToEeprom (void)
{
  const size_t sizeToSave = (uint32_t) &(servoStates[0].mutex) - (uint32_t) &servoStates[0];
  ErrorCond retVal = PROG_OK;

#ifdef	TRACE_PWM
  DebugTrace ("sizeToSave=%d, sizeof(servoStates[0])=%d", sizeToSave, sizeof(servoStates[0]));
  chThdSleepMilliseconds(100);
#endif //TRACE_PWM

  uint8_t *buffer = malloc_m (sizeToSave*SERVO_COUNT);
  if (buffer == NULL) {
    return MALLOC_ERR;
  }
  
  uint8_t *ptrFrom= (uint8_t *) servoStates;
  uint8_t *ptrTo=  buffer;

  for (uint32_t i=0; i<SERVO_COUNT; i++) {
#ifdef	TRACE_PWM
    DebugTrace ("ptrFrom=0x%x, ptrTo=0x%x", ptrFrom, ptrTo);
    chThdSleepMilliseconds(100);
#endif //TRACE_PWM

    memcpy (ptrTo, ptrFrom, sizeToSave);
    ptrFrom += sizeof(servoStates[0]);
    ptrTo += sizeToSave;
  }

  retVal = eepromStore (EEPROM_SERVO, buffer, sizeToSave*SERVO_COUNT);
  free_m (buffer);

  return retVal;
}

ErrorCond getServoStatesFromEeprom (void)
{
  const size_t sizeToRestore = (uint32_t) &(servoStates[0].mutex) - (uint32_t) &servoStates[0];
  uint8_t *buffer = malloc_m (sizeToRestore*SERVO_COUNT);
  ErrorCond retVal = PROG_OK;


  if (buffer == NULL) {
    return MALLOC_ERR;
  }

  uint8_t *ptrFrom=  buffer;
  uint8_t *ptrTo= (uint8_t *) servoStates;

  retVal = eepromLoad (EEPROM_SERVO, buffer, sizeToRestore*SERVO_COUNT);
  if ((retVal ==  PROG_OK) || (retVal ==  PROGRAM_MODIFIED)) {
    for (uint32_t i=0; i<SERVO_COUNT; i++) {
      chMtxLock (&(servoStates[0].mutex));
      memcpy (ptrTo, ptrFrom, sizeToRestore);
      chMtxUnlock();
      ptrFrom+=sizeToRestore;
      ptrTo+=sizeof(servoStates[0]);
    }
  }

  free_m (buffer);
  return retVal;
}


__attribute__((noreturn)) msg_t thdPwm(void *arg)
{
  (void) arg;
  chRegSetThreadName("pwm");
  while (TRUE) {
    if (isServoEngaged(SERVO_ALL_SERVOS)) {
      bb_palClearPad(GPIOF, GPIOF_LED_RED);
    } else {
      bb_palSetPad(GPIOF, GPIOF_LED_RED);
    }
    for (uint32_t i=0; i<SERVO_COUNT; i++) {
      servoPeriodicMove (i, FLOAT_SECOND_FROM_MS(TIME_STEP));
    }
    chThdSleepMilliseconds(TIME_STEP);
  }
}

