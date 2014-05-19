#include "calibration.h"
#include "ch.h"
#include "hal.h"
#include "stdutil.h"
#include "servo_pwm.h"
#include "analogicIO.h"
#include "IOmode.h"
#include "lcdDisplay.h"
#include "globalVar.h"
#include "printf.h"
#include <string.h>

#define MIDDLE_PWM		  1500
#define RANGE_PWM_FOR_STOP_SEARCH 700
#define PWM_SEARCH_NB_STEP	  100
#define PWM_STALE_TIME_MS	  50


#define MIN_PWM_FOR_STOP_SEARCH	  (MIDDLE_PWM-RANGE_PWM_FOR_STOP_SEARCH)
#define MAX_PWM_FOR_STOP_SEARCH	  (MIDDLE_PWM+RANGE_PWM_FOR_STOP_SEARCH)
#define PWM_SEARCH_STEP  (RANGE_PWM_FOR_STOP_SEARCH / PWM_SEARCH_NB_STEP)



// table, key : position read from servo slide resistor [0 ..1]
//      value : pwm order associated at calibration time
static uint16_t pwmCalibrationTable[SERVO_COUNT] [CALIBRATION_TABLE_SIZE];
static void setPosToMiddleSlowly (const uint32_t servoIdx, 
				    const uint32_t curPwm);
static void  setPwmByPos (const uint32_t servoIdx, const float pos, 
			    const uint16_t pwm);
static void fillHoleInTable  (void);
static void fillGapWithInterpoletedValues (const uint32_t servoIdx, const uint32_t startOfHoleIdx);

void calibratePwmRange (void)
{

  if ((getMode() != Mode_Tuning) && (getMode() != IO_mode_Off)) {
    syslog (LOG_ERROR, "calib impossible si");
    syslog (LOG_ERROR, "mode != reglage");
  }

  // reset all table values to zero
  memset (pwmCalibrationTable, 0, sizeof (pwmCalibrationTable));
  for (uint32_t srvIdx=0; srvIdx< SERVO_COUNT; srvIdx++) {
    calibratePwmServo (srvIdx);
  }
  fillHoleInTable();

 syslog (LOG_INFO, "calib terminee");
}


void calibratePwmServo (const uint32_t srvIdx)
{
  // set all other servo than the one to be calibrated to an idle
  // position where arm position will stay free of collision
  for (uint32_t j=0; j< SERVO_COUNT; j++) {
    setPwmValueByIndex (j, MIDDLE_PWM);
    chThdSleepMilliseconds (100);
  }
  chThdSleepMilliseconds (1000);
  // then search slowly from middle pos to MIN,
  // monitoring current consumption
  for (uint16_t pwm=MIDDLE_PWM; pwm >= MIN_PWM_FOR_STOP_SEARCH; 
       pwm -= PWM_SEARCH_STEP) {
    setPwmValueByIndex (srvIdx, pwm);
    chprintf (chp, "search MIN servo[%d] pwm=%d\r\n", srvIdx, pwm);
    chThdSleepMilliseconds (PWM_STALE_TIME_MS);
    const float currentIntensity =  analogGetCurrentIntensity (srvIdx);
    const float servoPos = analogGetRawPos(srvIdx);
    setPwmByPos (srvIdx, servoPos, pwm);
    if (currentIntensity >= max_current_intensity_calib[srvIdx]) {
      // return slowly to middle pos
      setPosToMiddleSlowly (srvIdx, pwm);
      servoSetPwmMin (srvIdx, pwm);
      break;
    }
  }


  // then search slowly from middle pos to MAX,
  // monitoring current consumption
  for (uint16_t pwm=MIDDLE_PWM; pwm <= MAX_PWM_FOR_STOP_SEARCH; 
       pwm += PWM_SEARCH_STEP) {
    setPwmValueByIndex (srvIdx, pwm);
    chprintf (chp, "search MAX servo[%d] pwm=%d\r\n", srvIdx, pwm);
    chThdSleepMilliseconds (PWM_STALE_TIME_MS);
    const float currentIntensity =  analogGetCurrentIntensity (srvIdx);
    const float servoPos = analogGetRawPos(srvIdx);
    setPwmByPos (srvIdx, servoPos, pwm);
    if (currentIntensity >= max_current_intensity_calib[srvIdx]) {
      servoSetPwmMax (srvIdx, pwm);
      // return slowly to middle pos
      setPosToMiddleSlowly (srvIdx, pwm);
      // then repeat search for next servo
      break;
    }
  }
  
}

 


uint16_t getPwmByReadPos (const uint32_t servoIdx, const float pos)
{
  if ((servoIdx) >= SERVO_COUNT) {
    return 0;
  }

  const uint16_t index = (uint16_t) ((INRANGE (0.f ,1.f, pos)) *  
				     CALIBRATION_TABLE_SIZE);
  return pwmCalibrationTable[servoIdx] [index];
}

static void  setPwmByPos (const uint32_t servoIdx, const float pos, 
			  const uint16_t pwm)
{
  if ((servoIdx) >= SERVO_COUNT) {
    return ;
  }

  const uint16_t index = (uint16_t) ((INRANGE (0.f ,1.f, pos)) *  
				     CALIBRATION_TABLE_SIZE);
  pwmCalibrationTable[servoIdx] [index] = pwm;
}

ErrorCond  storePwmByPosToEeprom (void)
{
  const size_t sizeToSave = sizeof (pwmCalibrationTable);
  ErrorCond retVal = eepromStore (EEPROM_PWM_CALIBRATION, pwmCalibrationTable, 
				  sizeToSave);
  
  return retVal;
}


ErrorCond  getPwmByPosFromEeprom (void)
{
  const size_t sizeToRestore = sizeof (pwmCalibrationTable);
  uint8_t *buffer = malloc_m (sizeToRestore);
  ErrorCond retVal = PROG_OK;


  if (buffer == NULL) {
    return MALLOC_ERR;
  }


  retVal = eepromLoad (EEPROM_PWM_CALIBRATION, buffer, sizeToRestore);
  if ((retVal ==  PROG_OK) || (retVal ==  PROGRAM_MODIFIED)) {
      memcpy (pwmCalibrationTable, buffer, sizeToRestore);
  }

  free_m (buffer);
  return retVal;
 }

 static void setPosToMiddleSlowly (const uint32_t servoIdx, 
				   const uint32_t curPwm)
 {
   const int16_t pwmStep  =  (curPwm < MIDDLE_PWM) ? PWM_SEARCH_STEP :
     -  PWM_SEARCH_STEP ;
   
   for (uint16_t pwm=curPwm; pwm != MIDDLE_PWM; pwm += pwmStep) {
     setPwmValueByIndex (servoIdx, pwm);
     chprintf (chp, "return to middle on servo[%d] pwm=%d\n\r", servoIdx, pwm);
     chThdSleepMilliseconds (PWM_STALE_TIME_MS);
   }
 }				   

static void fillHoleInTable  (void)
{
  for (uint32_t idx=0; idx<SERVO_COUNT; idx++) {
    uint32_t firstNonNullIdx=CALIBRATION_TABLE_SIZE;
    uint32_t lastNonNullIdx=CALIBRATION_TABLE_SIZE;
    // fill holes at start of table : find first non null value and copy this value
    // back to the start of table

    // find first valid data from start
    for (uint32_t i=0; i<CALIBRATION_TABLE_SIZE; i++) {
      if (pwmCalibrationTable[idx][i] != 0) {
	firstNonNullIdx=i;
	break;
      }
    }
    // find first last valid data to end
    for (uint32_t i=CALIBRATION_TABLE_SIZE; i-- != 0;) {
      if (pwmCalibrationTable[idx][i] != 0) {
	lastNonNullIdx=i;
	break;
      }
    }

    // fill holes at start
    for (uint32_t i=0; i<firstNonNullIdx; i++) {
      pwmCalibrationTable[idx][i] = pwmCalibrationTable[idx][firstNonNullIdx];
    }

    // fill holes at end
    for (uint32_t i=CALIBRATION_TABLE_SIZE-1; i>lastNonNullIdx; i--) {
      pwmCalibrationTable[idx][i] = pwmCalibrationTable[idx][lastNonNullIdx];
    }

    // fill gaps in data
    for (uint32_t i=0; i<CALIBRATION_TABLE_SIZE; i++) {
      if (pwmCalibrationTable[idx][i] == 0) {
	fillGapWithInterpoletedValues (idx, i);
      }
    }
  }
}

static void fillGapWithInterpoletedValues (const uint32_t servoIdx, const uint32_t startOfHoleIdx) 
{
  if (startOfHoleIdx == 0) 
    return;

  // find end of hole
  uint32_t endOfHoleIdx=0;
  for (uint32_t i=startOfHoleIdx; i<CALIBRATION_TABLE_SIZE; i++) {
    if (pwmCalibrationTable[servoIdx][i] != 0) {
      endOfHoleIdx=i;
      break;
    }
  }
  if (endOfHoleIdx==0)
    return;

  const uint32_t startValidIdx=startOfHoleIdx-1;
  const uint32_t endValidIdx=endOfHoleIdx;

  for (uint32_t i=startOfHoleIdx; i<endValidIdx; i++) {
    pwmCalibrationTable[servoIdx][i] = 
      (uint16_t) lerpf (pwmCalibrationTable[servoIdx][startValidIdx], 
			pwmCalibrationTable[servoIdx][endOfHoleIdx], 
			(float) (i-startValidIdx) /(float) (endOfHoleIdx - startValidIdx));
  }
}

float getErrorRatio (const uint32_t servoIdx)
{
   if ((servoIdx) >= SERVO_COUNT) {
    return 0.f;
  }


  const float cmdPwm = getPwmValueByIndex (servoIdx);
  const float readPwm = getPwmByReadPos (servoIdx, analogGetRawPos (servoIdx));
  return (readPwm / cmdPwm) - 1.0;
}


float   getOrderByReadPos (const uint32_t servoIdx, const float pos)
{
  const uint16_t pwm = getPwmByReadPos (servoIdx, pos);

  return unlerpf (servoGetPwmMin(servoIdx), servoGetPwmMax(servoIdx), pwm);
}

float   getReadPosNormalised (const uint32_t servoIdx)
{
  return getOrderByReadPos (servoIdx, analogGetRawPos (servoIdx));
}

bool_t     isCalibrated(void)
{
  bool_t retVal = TRUE;

  for (uint32_t idx=0; idx<SERVO_COUNT; idx++) {
    if (pwmCalibrationTable[idx][0] == 0) {
      retVal = FALSE;
      break;
    }
  }

  return retVal;
}

