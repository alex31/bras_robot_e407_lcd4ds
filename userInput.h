#pragma once
#include "ch.h"
#include "hal.h"

typedef enum {
  Button_Press, Button_Release, Button_None
}  ButtonEvent;


Thread* launchInputKnob (void);
Thread* launchInputJoystick (void);
bool_t isDaughterBoardPlugged (void);
bool_t isLcdBoardPlugged (void);
bool_t isArmBoardPlugged (void);
uint8_t getRotaryAntiBouncedKnobPos (void);

