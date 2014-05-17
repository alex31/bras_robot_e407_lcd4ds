#ifndef _LOGIC_LEVEL_IO_H_
#define _LOGIC_LEVEL_IO_H_
#include "servo_pwm.h"
#include "hal.h"

#define MOVE_IN_BIT (0x1)
#define BACKWARD_BIT (0x10)


void initLogicLevelIO (void);
uint32_t getInputs (uint32_t servoIdx);
msg_t thdLogicLevelIO(void *arg);

#endif /*_LOGIC_LEVEL_IO_H_ */
