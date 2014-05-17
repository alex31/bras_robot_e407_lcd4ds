#ifndef _ANALOGIC_IO_H_
#define _ANALOGIC_IO_H_
#include "servo_pwm.h"
#include "hal.h"

#define MOVE_IN_BIT (0x1)
#define BACKWARD_BIT (0x10)



float analogGetCmd (uint32_t servoIdx); // lecture de la conversion ADC de la commande
float analogGetCurrentIntensity (uint32_t servoIdx); // lecture de la conversion ADC de la commande
void  analogCalibrateCurrentIntensity (void); // should be called when motor 
					      // are not active : no  pwm output
float analogGetRawPos (uint32_t servoIdx); //  lecture de la conversion ADC 
					// du potentiometre du servomoteur via chip i2c
					// valeur de retour normalisée [0.0 .. 1.0]

// deduce regulated 5 volts power voltage from the measure of a 3.3v 
// precision voltage source. Should always stay around 5V otherwise
// it would be that 5V power source is not able to feed servos
float analogGet5VoltPowerVoltage (void);


msg_t thdAnalogIO(void *arg);

#endif /*_ANALOGIC_IO_H_ */
