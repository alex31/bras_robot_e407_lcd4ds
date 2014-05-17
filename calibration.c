#include "calibration.h"
#include "ch.h"
#include "hal.h"
#include "stdutil.h"
#include "servo_pwm.h"
#include "analogicIO.h"
#include <string.h>

#define CALIBRATION_TABLE_SIZE	  512
#define MIDDLE_PWM		  1500
#define RANGE_PWM_FOR_STOP_SEARCH 700
#define PWM_SEARCH_NB_STEP	  100
#define PWM_STALE_TIME_MS	  30


#define MIN_PWM_FOR_STOP_SEARCH	  (MIDDLE_PWM-RANGE_PWM_FOR_STOP_SEARCH)
#define MAX_PWM_FOR_STOP_SEARCH	  (MIDDLE_PWM+RANGE_PWM_FOR_STOP_SEARCH)
#define PWM_SEARCH_STEP  (RANGE_PWM_FOR_STOP_SEARCH/ PWM_SEARCH_NB_STEP)


static uint16_t pwmCalibrationTable[SERVO_COUNT] [CALIBRATION_TABLE_SIZE];
static void setPosToMiddleSlowly (const uint32_t servoIdx, 
				    const uint32_t curPwm);
static void  setPwmByPos (const uint32_t servoIdx, const float pos, 
			    const uint16_t pwm);




void calibratePwmRange (void)
{
  // reset all table values to zero
  memset (pwmCalibrationTable, 0, sizeof (pwmCalibrationTable));
  for (uint32_t srvIdx=0; srvIdx< SERVO_COUNT; srvIdx++) {
    calibratePwmServo (srvIdx);
  }
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
    chThdSleepMilliseconds (PWM_STALE_TIME_MS);
    const float currentIntensity =  analogGetCurrentIntensity (srvIdx);
    const float servoPos = analogGetRawPos(srvIdx);
    setPwmByPos (srvIdx, servoPos, pwm);
    if (currentIntensity >= max_current_intensity_calib[srvIdx]) {
      servoSetPwmMin (srvIdx, pwm);
      // return slowly to middle pos
      setPosToMiddleSlowly (srvIdx, pwm);
      // then search slowly from middle pos to MAN,
      // monitoring current consumption
      
      for (uint16_t pwm=MIDDLE_PWM; pwm <= MAX_PWM_FOR_STOP_SEARCH; 
	   pwm += PWM_SEARCH_STEP) {
	setPwmValueByIndex (srvIdx, pwm);
	chThdSleepMilliseconds (PWM_STALE_TIME_MS);
	const float currentIntensity =  analogGetCurrentIntensity (srvIdx);
	const float servoPos = analogGetRawPos(srvIdx);
	setPwmByPos (srvIdx, servoPos, pwm);
	if (currentIntensity >= max_current_intensity_calib[srvIdx]) {
	  servoSetPwmMax (srvIdx, pwm);
	  // return slowly to middle pos
	  setPosToMiddleSlowly (srvIdx, pwm);
	  // then repeat search for next servo
	  return;
	}
      }
    }
  }
}
 


uint16_t getPwmByPos (const uint32_t servoIdx, const float pos)
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
     chThdSleepMilliseconds (PWM_STALE_TIME_MS);
   }
 }				   


float getErrorPercentage (const uint32_t servoIdx)
{
   if ((servoIdx) >= SERVO_COUNT) {
    return 0.f;
  }


  const float cmdPwm = getPwmValueByIndex (servoIdx);
  const float readPwm = getPwmByPos (servoIdx, analogGetRawPos (servoIdx));
  return (readPwm / cmdPwm) - 1.0;
}
