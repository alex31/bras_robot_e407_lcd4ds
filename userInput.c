#include "userInput.h"

#include "ch.h"
#include "hal.h"
#include "printf.h"
#include "IOmode.h"
#include "lcdDisplay.h"
#include "servo_pwm.h"
#include "bitband.h"
#ifdef WATCHDOG_ACTIVATED
#include "stm32f4xx_iwdg.h"
#endif

static WORKING_AREA(waThdModeInputKnob, 1024);
static WORKING_AREA(waThdModeInputJoystick, 512);
static __attribute__((noreturn)) msg_t thdModeInputKnob(void *arg) ;
static __attribute__((noreturn)) msg_t thdModeInputJoystick(void *arg) ;


#define BUTTON_GREEN (!bb_palReadPad (GPIOE, GPIOE_BUTTON_GREEN))
#define BUTTON_RED (!bb_palReadPad (GPIOE, GPIOE_BUTTON_RED))
#define JOYSTICK_UP (!bb_palReadPad (GPIOD, GPIOD_JOYSTICK_UP))
#define JOYSTICK_DOWN (!bb_palReadPad (GPIOD, GPIOD_JOYSTICK_DOWN))
#define JOYSTICK_RIGHT (!bb_palReadPad (GPIOD, GPIOD_JOYSTICK_RIGHT))
#define JOYSTICK_LEFT (!bb_palReadPad (GPIOD, GPIOD_JOYSTICK_LEFT))
#define JOYSTICK_CLICK (!bb_palReadPad (GPIOD, GPIOD_JOYSTICK_CLICK))



Thread* launchInputKnob (void)
{
  return chThdCreateStatic(waThdModeInputKnob, sizeof(waThdModeInputKnob),
			   NORMALPRIO, thdModeInputKnob, NULL);}


Thread* launchInputJoystick (void)
{
  return chThdCreateStatic(waThdModeInputJoystick, sizeof(waThdModeInputJoystick),
			   NORMALPRIO, thdModeInputJoystick, NULL);
}


static __attribute__((noreturn)) msg_t thdModeInputKnob(void *arg) 
{
  static const enum IO_modes modeByRotaryPos[] = {IO_mode_Off, Mode_Logic, Mode_Analog_Pos, Mode_Jbus485, 
						  Mode_JbusEther,  Mode_Tuning};
  (void)arg;
  chRegSetThreadName("Mode_input_knob");
  
  while (TRUE) {
    //    setMode (Mode_Logic);
    //    chThdSleepMilliseconds(1000 * 1000); 
    if (isLcdBoardPlugged()) 
      setMode (modeByRotaryPos[getRotaryAntiBouncedKnobPos()]);
    else
      // just for the watchdog reset timer reload
      getRotaryAntiBouncedKnobPos();
  }
}


#define CHECK_BUTTON_EVENT(e) (event[e]=checkButtonStateForEvent (e, &(_state[GPIOE_ ## e])))
#define CHECK_JOYSTICK_EVENT(e) (event[e]=checkButtonStateForEvent (e, &(_state[GPIOD_ ## e])))

static ButtonEvent checkButtonStateForEvent (const bool_t newState, bool_t * const lastState)
{
  if (newState == *lastState) {
    return Button_None;
  } else {
    *lastState = newState;
    if (newState == TRUE) {
      return Button_Press ;
    } else {
      return Button_Release;
    }
  }
}

static __attribute__((noreturn)) msg_t thdModeInputJoystick(void *arg) 
{
  (void)arg;
  chRegSetThreadName("Mode_input_joystick");
  bool_t _state[16] = {0};
  ButtonEvent event[16];

  while (TRUE) {
    CHECK_BUTTON_EVENT(BUTTON_GREEN);
    CHECK_BUTTON_EVENT(BUTTON_RED);
    CHECK_JOYSTICK_EVENT(JOYSTICK_UP);
    CHECK_JOYSTICK_EVENT(JOYSTICK_DOWN);
    CHECK_JOYSTICK_EVENT(JOYSTICK_LEFT);
    CHECK_JOYSTICK_EVENT(JOYSTICK_RIGHT);
    CHECK_JOYSTICK_EVENT(JOYSTICK_CLICK);

    if (event[JOYSTICK_UP] == Button_Press)
      setSysLogMode (SYSLOG);
    else if (event[JOYSTICK_DOWN] == Button_Press)
      setSysLogMode (SPLIT_PAGE);
 
    if (event[BUTTON_GREEN] == Button_Press) {
      servoEngage (SERVO_ALL_SERVOS);
      syslog (LOG_INFO, "Servos ** ON **");
	} else if (event[BUTTON_RED] == Button_Press) {
      servoDisengage (SERVO_ALL_SERVOS);
      syslog (LOG_WARN, "Servos ** OFF **");
      }


    if (getMode () == Mode_Tuning) {
      if (event[JOYSTICK_UP]  == Button_Release) {
	syslog (LOG_INFO, "JOYSTICK_UP release");
	bb_palTogglePad (GPIOF, GPIOF_LED_RED);
      } else if (event[JOYSTICK_DOWN]  == Button_Release) {
	syslog (LOG_INFO, "JOYSTICK_DOWN release");
	bb_palTogglePad (GPIOF, GPIOF_LED_RED);
      } else if (event[JOYSTICK_LEFT] == Button_Press) {
	syslog (LOG_INFO, "JOYSTICK_LEFT");
	bb_palTogglePad (GPIOF, GPIOF_LED_RED);
      } else if (event[JOYSTICK_RIGHT] == Button_Press) {
	syslog (LOG_INFO, "JOYSTICK_RIGHT");
	bb_palTogglePad (GPIOF, GPIOF_LED_RED);
      } else if (event[JOYSTICK_CLICK] == Button_Press) {
	syslog (LOG_INFO, "JOYSTICK_CLICK");
	bb_palTogglePad (GPIOF, GPIOF_LED_RED);
      }
    }

    chThdSleepMilliseconds(10);  
  }
}


// Daughter boards plug are detected by setting VCC to probe pin  GPIOE_*_PLUGGED

bool_t isDaughterBoardPlugged (void)
{
  palSetPadMode (GPIOE, GPIOE_DAUGHTERBOARD_PLUGGED,
		 PAL_MODE_INPUT_PULLDOWN);
  
  const bool_t attached = bb_palReadPad (GPIOE, GPIOE_DAUGHTERBOARD_PLUGGED);
  return attached;
}

bool_t isLcdBoardPlugged (void)
{
  palSetPadMode (GPIOE, GPIOE_LCD_BOARD_PLUGGED,
		 PAL_MODE_INPUT_PULLDOWN);
  
  const bool_t attached = bb_palReadPad (GPIOE, GPIOE_LCD_BOARD_PLUGGED);
  return attached;
}

bool_t isArmBoardPlugged (void)
{
  palSetPadMode (GPIOE, GPIOE_ARM_BOARD_PLUGGED,
		 PAL_MODE_INPUT_PULLDOWN);
  
  const bool_t attached = bb_palReadPad (GPIOE, GPIOE_ARM_BOARD_PLUGGED);
  return attached;
}

static uint8_t getRotaryKnobPos (void)
{
  static const uint8_t knobGpioPin[] = {GPIOD_KNOB_OFF, GPIOD_KNOB_LVTTL,
					GPIOD_KNOB_ANALOG, GPIOD_KNOB_RS485,
					GPIOD_KNOB_ETHERNET, GPIOD_KNOB_TUNING};

  for (uint8_t i=0; i< sizeof(knobGpioPin); i++) {
    if (!bb_palReadPad (GPIOD, knobGpioPin[i])) 
      return i;
  }

  return 0;
}

uint8_t getRotaryAntiBouncedKnobPos (void)
{
  const int8_t conseqSamePosInitial = 5;
  int8_t conseqSamePos = conseqSamePosInitial;
  uint8_t pos = 0;

  do {
    if (pos != getRotaryKnobPos ())
      conseqSamePos=conseqSamePosInitial;
    else 
      conseqSamePos--;
    
    pos = getRotaryKnobPos ();
    chThdSleepMilliseconds(20);    
#ifdef WATCHDOG_ACTIVATED
    IWDG_ReloadCounter();  
#endif
  } while (conseqSamePos >= 0);
  
  return pos;
}
