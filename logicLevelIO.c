#include "stdutil.h"
#include "logicLevelIO.h"
#include "IOmode.h"
#include "globalVar.h"
#include "printf.h"
#include "bitband.h"



struct LogicLevelIO_ {
  GpioPin move_in;
  GpioPin fwdOrBwd_in;
  GpioPin stop_Low;
  GpioPin stop_High;
} ;
typedef struct LogicLevelIO_ LogicLevelIO;


#if defined BOARD_STM32_E407_STX

static const LogicLevelIO logicLevelIOS[] = {  
  {.move_in = {.gpio = GPIOE, .pin = GPIOE_CMD_TURRET},
   .fwdOrBwd_in = {.gpio = GPIOE, .pin = GPIOE_CMD_TURRET_DIR},
   .stop_Low = {.gpio = GPIOF, .pin = GPIOF_STOP_TURRET_RIGHT},
   .stop_High= {.gpio = GPIOG, .pin = GPIOG_STOP_TURRET_LEFT}},

  {.move_in = {.gpio = GPIOE, .pin = GPIOE_CMD_SHOULDER},
   .fwdOrBwd_in = {.gpio = GPIOE, .pin = GPIOE_CMD_SHOULDER_DIR},
   .stop_Low = {.gpio = GPIOG, .pin = GPIOG_STOP_SHOULDER_BACK},
   .stop_High= {.gpio = GPIOG, .pin = GPIOG_STOP_SHOULDER_FRONT}},
  
  {.move_in = {.gpio = GPIOE, .pin = GPIOE_CMD_ELBOW},
   .fwdOrBwd_in = {.gpio = GPIOE, .pin = GPIOE_CMD_ELBOW_DIR},
   .stop_Low = {.gpio = GPIOG, .pin = GPIOG_STOP_ELBOW_HIGH},
   .stop_High= {.gpio = GPIOG, .pin = GPIOG_STOP_ELBOW_LOW}},

  {.move_in = {.gpio = GPIOE, .pin = GPIOE_CMD_WRIST},
   .fwdOrBwd_in = {.gpio = GPIOE, .pin =  GPIOE_CMD_WRIST_DIR},
   .stop_Low = {.gpio = GPIOG, .pin = GPIOG_STOP_WRIST_HIGH},
   .stop_High= {.gpio = GPIOG, .pin = GPIOG_STOP_WRIST_LOW}},

  {.move_in = {.gpio = GPIOE, .pin = GPIOE_CMD_CLAMP},
   .fwdOrBwd_in = {.gpio = GPIOE, .pin = GPIOE_CMD_CLAMP_DIR},
   .stop_Low = {.gpio = GPIOG, .pin = GPIOG_STOP_CLAMP_OPEN},
   .stop_High= {.gpio = GPIOG, .pin = GPIOG_STOP_CLAMP_CLOSED}},
};

#else
#error "only compatible with BOARD_STM32_E407_STX"
#endif

#ifndef __COVERITY__
_Static_assert(  ARRAY_LEN(logicLevelIOS) == SERVO_COUNT, 
		"logicLevelIOS should be array of same"
		" length than macro SERVO_COUNT defined in servo_pwm.h");

#endif

/* PAL_MODE_RESET */
/* PAL_MODE_UNCONNECTED */
/* PAL_MODE_INPUT */
/* PAL_MODE_INPUT_PULLUP */
/* PAL_MODE_INPUT_PULLDOWN */
/* PAL_MODE_INPUT_ANALOG */
/* PAL_MODE_OUTPUT_PUSHPULL */
/* PAL_MODE_OUTPUT_OPENDRAIN */
/* PAL_MODE_ALTERNATE(n) */
/* see os/hal/platforms/STM32/GPIOv2/pal_lld.h */

void initLogicLevelIO (void)
{
  for (uint32_t i=0; i<SERVO_COUNT; i++) {
    palSetPadMode (logicLevelIOS[i].move_in.gpio, 
		   logicLevelIOS[i].move_in.pin, 
		   PAL_MODE_INPUT_PULLUP); 
    palSetPadMode (logicLevelIOS[i].fwdOrBwd_in.gpio, 
		   logicLevelIOS[i].fwdOrBwd_in.pin, 
		   PAL_MODE_INPUT_PULLUP); 
    palSetPadMode (logicLevelIOS[i].stop_Low.gpio, 
		   logicLevelIOS[i].stop_Low.pin, 
		   PAL_MODE_OUTPUT_PUSHPULL); 
    palSetPadMode (logicLevelIOS[i].stop_High.gpio, 
		   logicLevelIOS[i].stop_High.pin, 
		   PAL_MODE_OUTPUT_PUSHPULL); 
  }
}



uint32_t getInputs (uint32_t servoIdx)
{
  uint32_t  retVal = 0;
  chSysLock();
  if (!bb_palReadPad (logicLevelIOS[servoIdx].move_in.gpio, 
		  logicLevelIOS[servoIdx].move_in.pin)) {
    retVal |= MOVE_IN_BIT;
  }

  if (bb_palReadPad (logicLevelIOS[servoIdx].fwdOrBwd_in.gpio, 
		  logicLevelIOS[servoIdx].fwdOrBwd_in.pin)) {
    retVal |= BACKWARD_BIT;
  }
  chSysUnlock();
  
  return retVal;
}


static void setLowStopOut (uint32_t servoIdx, uint32_t level)
{
  if (level) {
    bb_palSetPad (logicLevelIOS[servoIdx].stop_Low.gpio, 
	       logicLevelIOS[servoIdx].stop_Low.pin);
    /* if (chTimeNow() > 5000) { */
    /*   chprintf (chp, "Low stop %d\r\n", servoIdx); */
    /* } */
  } else {
    bb_palClearPad (logicLevelIOS[servoIdx].stop_Low.gpio, 
		 logicLevelIOS[servoIdx].stop_Low.pin);
   /* if (chTimeNow() > 5000) { */
   /*    chprintf (chp, "Clear low stop %d\r\n", servoIdx); */
   /*  } */
  }
}

static void setHighStopOut (uint32_t servoIdx, uint32_t level)
{
  if (level) {
    bb_palSetPad (logicLevelIOS[servoIdx].stop_High.gpio, 
	       logicLevelIOS[servoIdx].stop_High.pin);
   /* if (chTimeNow() > 5000)  { */
   /*   chprintf (chp, "High stop %d\r\n", servoIdx); */
   /*  } */
    
  } else {
    bb_palClearPad (logicLevelIOS[servoIdx].stop_High.gpio, 
		 logicLevelIOS[servoIdx].stop_High.pin);
    /* if (chTimeNow() > 5000) { */
    /*   chprintf (chp, "Clear high stop %d\r\n", servoIdx); */
    /* } */
  }
}

/*
 * logical level switch I/O thread
 */
msg_t thdLogicLevelIO(void *arg) 
{
  (void)arg;

  chRegSetThreadName("thdLogicLevelIO");
 
  /*
   * initialize the Logic IO (15 pins)
   */
  initLogicLevelIO ();

  while (!chThdShouldTerminate()) {
    for (uint32_t i=0; i<SERVO_COUNT; i++) {
      const uint32_t input =   getInputs (i);
      if (isActive(Logic_in)) {
	if (input & MOVE_IN_BIT)  {
	  if (input & BACKWARD_BIT) {
	    servoSlidePosTowardBegin(i, FLOAT_SECOND_FROM_MS(TIME_STEP));
	  } else {
	    servoSlidePosTowardEnd(i, FLOAT_SECOND_FROM_MS(TIME_STEP));
	  }
	}
      }

 
      if (isActive(Logic_out)) {
	setLowStopOut (i,  isAtLowStopPos (i));
	setHighStopOut (i,  isAtHighStopPos (i));
      } else {
	/* if (chTimeNow() > 5000) { */
	/*   chprintf (chp, "Logic_out not active\r\n"); */
	/* } */
      }
    }
    
    chThdSleepMilliseconds(TIME_STEP);
  }
  
  return 0;
}
