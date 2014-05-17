#ifndef __SERVO_PWM_H__
#define __SERVO_PWM_H__

#include "ch.h"
#include "eeprom.h"



#define SERVO_TURRET_IDX 0
#define SERVO_SHOULDER_IDX 1
#define SERVO_ELBOW_IDX 2
#define SERVO_WRIST_IDX 3
#define SERVO_CLAMP_IDX 4
#define	SERVO_COUNT 5
#define	SERVO_ALL_SERVOS (SERVO_COUNT+1)

extern const float max_current_intensity_calib[SERVO_COUNT];
extern const float max_current_intensity_use[SERVO_COUNT];

#define FLOAT_SECOND_FROM_MS(ms) ((float) ms / 1000.0f)
//#define TIME_STEP 100 // in millisecond for debug
#define TIME_STEP 20  // in millisecond for real world


typedef struct PwmServo_ PwmServo;
typedef enum {ADS_Idle=0, ADS_Jam, ADS_Accelerate, ADS_TopSpeed, ADS_Decelerate} 
  AccelDecelState;	

// public method
void servoInit (const uint32_t servoIdx);
void initAllServos (void);
void servoSetPos (const uint32_t servoIdx, float order);
void servoSlidePosTowardEnd (const uint32_t servoIdx, float timeInSecond);
void servoSlidePosTowardBegin (const uint32_t servoIdx, float timeInSecond);
void servoSetPos (const uint32_t servoIdx, float order);
void servoPark (const uint32_t servoIdx);
void servoSetTopSpeed (const uint32_t servoIdx, float speed);
void servoSetPwmMin (const uint32_t servoIdx, uint32_t pwmVal);
void servoSetPwmMax (const uint32_t servoIdx, uint32_t pwmVal);
void inversePwmMinAndMax  (const uint32_t servoIdx);
float servoGetPos (const uint32_t servoIdx);
float servoGetParkPos (const uint32_t servoIdx);
float servoGetCurrentPos (const uint32_t servoIdx);
float servoGetTopSpeed (const uint32_t servoIdx);
float servoGetCurrentSpeed (const uint32_t servoIdx);
AccelDecelState servoGetState (const uint32_t servoIdx);
bool_t isServoMoving (const uint32_t servoIdx);
bool_t isAtLowStopPos (const uint32_t servoIdx);
bool_t isAtHighStopPos (const uint32_t servoIdx);
bool_t isFadeInOrOut (const uint32_t servoIdx);
uint32_t servoGetPwmMin (const uint32_t servoIdx);
uint32_t servoGetPwmMax (const uint32_t servoIdx);
void servoSetCurrentPosAsPark (const uint32_t servoIdx);
void setPwmValueByIndex (const uint32_t servoIdx, const uint32_t value);
uint32_t getPwmValueByIndex (const uint32_t servoIdx);
void servoEnableDrive (const uint32_t servoIdx);
void servoPeriodicMove (const uint32_t servoIdx, float timeInSecond);
void servoEngage (const uint32_t servoIdx);
void servoDisengage (const uint32_t servoIdx);
bool_t isServoEngaged (const uint32_t servoIdx);

ErrorCond getServoStatesFromEeprom (void);
ErrorCond storeServoStatesToEeprom (void);

// private method
void timerAndPwmInit (void) ;
void setPwmValue (const PwmServo *s, const uint32_t value);
__attribute__((noreturn)) msg_t thdPwm(void *arg);


#endif //  __SERVO_PWM_H__
