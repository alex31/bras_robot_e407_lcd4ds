#include "ch.h"
#include "hal.h"
#include "printf.h"
#include "stdutil.h"
#include "globalVar.h"
#include "servo_pwm.h"
#include "logicLevelIO.h"
#include "lcdDisplay.h"
#include "IOmode.h"
#include "i2cMaster.h" // I choose to mix native adc and i2c adc 
                       // in same file, yes, it's weird
#include <math.h>
#include <string.h>
#include <stdlib.h>
//#include <tgmath.h>

static const I2CConfig i2ccfg_100 = {
  .op_mode = OPMODE_I2C,
  .clock_speed = 100000,
  .duty_cycle = STD_DUTY_CYCLE
};

/* static const I2CConfig i2ccfg_400 = { */
/*   .op_mode = OPMODE_I2C, */
/*   .clock_speed = 400000, */
/*   .duty_cycle = FAST_DUTY_CYCLE_2 */
/* }; */

static const I2cMasterConfig i2c2 = {
  .driver = &I2CD2,
  .i2ccfg = &i2ccfg_100,
  .sdaGpio = GPIOF,    
  .sclGpio = GPIOF,  
  .sdaPin = GPIOF_I2C2_SDA,        
  .sclPin = GPIOF_I2C2_SCL,        
  .alternateFunction = 4,     
};

#define MIN_5VOLTS_POWER_SUPPLY_VOLTAGE (4.3f)
#define ACS712_ZERO_VOLTAGE (2.5f)
#define VCC_VOLTAGE (3.3f)
#define LOW_PASS_N (9)
#define I2C_ADC_SERVOPOS_ADR_OFFSET 0
#define I2C_ADC_CURRENT_ADR_OFFSET 1
#define I2C_MAX_CHANNELS 8

// PCLK2 = 42Mhz, with PREDIV=4, ADCCLK=10.5Mhz
// with 10 channels ADC_SAMPLE_480, sample time is (1/10.5e6)*480*10 = 0.5ms

// New version of board : Intensity is sampled via i2c instead of mcu ADC
// No ADC1

#define ADC_OVERSAMPLING   ADC_SAMPLE_480
#define ADC_BUF_DEPTH      4
#define MEDIAN_SIZE	   5

#define ADC_SERVO_POS      0
#define ADC_SERVO_CURRENT  1
#define ADC_VALUE_SORT     2
#define ADC_MEDIAN_SIZE    3

static adcsample_t samples3[ADC_ADC3_NUM_CHANNELS * ADC_BUF_DEPTH] __attribute__((aligned(8)));


/*
  order of data in array av :
  GPIOA_ADC_CURRENT_ELBOW_CHANNEL
  GPIOA_ADC_CURRENT_WRIST_CHANNEL
  GPIOA_ADC_CURRENT_CLAMP_CHANNEL
  GPIOF_ADC_CURRENT_TURRET
  GPIOF_ADC_CURRENT_SHOULDER
  GPIOF_ADC_CMD_TURRET
  GPIOF_ADC_CMD_SHOULDER
  GPIOF_ADC_CMD_ELBOW
  GPIOF_ADC_CMD_WRIST
  GPIOF_ADC_CMD_CLAMP
}
 */

static void initAnalogIO (void);
static void closeAnalogIO (void);
static int qsortCompareFloatCb (const void *a, const void *b);

static float currentCalibrateValue[SERVO_COUNT+1] = {
  [0 ... SERVO_COUNT-1] = ACS712_ZERO_VOLTAGE/VCC_VOLTAGE
};

static uint32_t av[ADC_ADC3_NUM_CHANNELS]; 
static float    i2cAvPos[I2C_MAX_CHANNELS]; // 5 servos pos + 3.3v ref
static float    i2cAvCurrent[I2C_MAX_CHANNELS]; // 5 servos current + foam resistivity



/*
 * ADC streaming callback.
 */
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) 
{
  (void) n;
  (void) buffer;
  uint32_t accum;

  if (adcp->state != ADC_COMPLETE) 
    return;
  
  static bool firstTime=TRUE;
  // initialize low pass filter to avoid long time convergence
  if (firstTime) {
    firstTime = FALSE;
    for (uint32_t i=0; i<ADC_ADC3_NUM_CHANNELS; i++) {
      accum = 0;
      for (uint32_t j=0; j< ADC_BUF_DEPTH; j++) {
	accum +=  samples3[i+(ADC_ADC3_NUM_CHANNELS*j)];
      }
      av[i] = accum / (ADC_BUF_DEPTH);
    }  
  }
  
  for (uint32_t i=0; i<ADC_ADC3_NUM_CHANNELS; i++) {
    accum = 0;
    for (uint32_t j=0; j< ADC_BUF_DEPTH; j++) {
      accum +=  samples3[i+(ADC_ADC3_NUM_CHANNELS*j)];
    }
    av[i] = ((av[i]*LOW_PASS_N*ADC_BUF_DEPTH) + accum) / 
      ((LOW_PASS_N+1)*ADC_BUF_DEPTH);
  }  
}



static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;

  DebugTrace ("adcerrorcallback, error=%d", err);
}


typedef struct  {
  GpioPin order_in;
  //  GpioPin armPos_in;
  //  GpioPin pos_out;
} AnalogIO;

#if defined BOARD_STM32_E407_STX

static const AnalogIO analogIOS[] = {  
  {.order_in = {.gpio = GPIOF, .pin = GPIOF_ADC_CMD_TURRET}},
  {.order_in = {.gpio = GPIOF, .pin = GPIOF_ADC_CMD_ELBOW}},
  {.order_in = {.gpio = GPIOF, .pin = GPIOF_ADC_CMD_SHOULDER}},
  {.order_in = {.gpio = GPIOF, .pin = GPIOF_ADC_CMD_WRIST}},
  {.order_in = {.gpio = GPIOF, .pin = GPIOF_ADC_CMD_CLAMP}},
};

/*
 * ADC conversion group.
 * Mode:        
 * Channels:    
 */


static const ADCConversionGroup adcgrpcfg3 = {
  FALSE, //circular enable
  ADC_ADC3_NUM_CHANNELS, //num_channels
  adccallback, // end_conversion_cb
  adcerrorcallback, //error_cb
  0,                        /* CR1 */ 
  ADC_CR2_SWSTART,          /* CR2 */ 

  ADC_SMPR1_SMP_AN14(ADC_OVERSAMPLING) | ADC_SMPR1_SMP_AN15(ADC_OVERSAMPLING), // SMPR1

  ADC_SMPR2_SMP_AN4(ADC_OVERSAMPLING) | ADC_SMPR2_SMP_AN5(ADC_OVERSAMPLING) | /* SMPR2 */ 
  ADC_SMPR2_SMP_AN6(ADC_OVERSAMPLING) | ADC_SMPR2_SMP_AN7(ADC_OVERSAMPLING) | /* SMPR2 */ 
  ADC_SMPR2_SMP_AN9(ADC_OVERSAMPLING),  /* SMPR2 */ 

  ADC_SQR1_NUM_CH(ADC_ADC3_NUM_CHANNELS), // SQR1

  0,					  //SQR2
  ADC_SQR3_SQ1_N(GPIOF_ADC_CMD_TURRET_CHANNEL) |	//SQR3
  ADC_SQR3_SQ2_N(GPIOF_ADC_CMD_ELBOW_CHANNEL) |		//SQR3
  ADC_SQR3_SQ3_N(GPIOF_ADC_CMD_SHOULDER_CHANNEL)   | 	//SQR3
  ADC_SQR3_SQ4_N(GPIOF_ADC_CMD_WRIST_CHANNEL) | 	//SQR3
  ADC_SQR3_SQ5_N(GPIOF_ADC_CMD_CLAMP_CHANNEL)	        //SQR3
};

#ifndef __COVERITY__
_Static_assert(CMD_TURRET_ADC_IDX == 3, "ADC mismatch");
_Static_assert(CMD_SHOULDER_ADC_IDX == 3, "ADC mismatch");
_Static_assert(CMD_ELBOW_ADC_IDX == 3, "ADC mismatch");
_Static_assert(CMD_WRIST_ADC_IDX == 3, "ADC mismatch");
_Static_assert(CMD_CLAMP_ADC_IDX == 3, "ADC mismatch");
#endif

#else
#error "one board has to be defined"
#endif

#ifndef __COVERITY__
_Static_assert(  ARRAY_LEN(analogIOS) == ADC_ADC3_NUM_CHANNELS,
		"analogIOS should be array of same"
		" length than macro ADC_ADCX_NUM_CHANNELS defined in board.h");

_Static_assert( SERVO_COUNT  == ADC_ADC3_NUM_CHANNELS,
		"ADC_ADC3_NUM_CHANNELS (analog position command) defined in board.h should be "
		"equal to SERVO_COUNT defined in servo_pwm.h");

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

void initAnalogIO (void)
{
  chSysLock ();
  for (uint32_t i=0; i<SERVO_COUNT; i++) {
    palSetPadMode (analogIOS[i].order_in.gpio, 
		   analogIOS[i].order_in.pin, 
		   PAL_MODE_INPUT_ANALOG); 
  }
  chSysUnlock ();

  adcStart (&ADCD3, NULL);
}

void closeAnalogIO (void)
{

  adcStopConversion (&ADCD3);
  chThdSleepMilliseconds(TIME_STEP);

  adcStop (&ADCD3);
}


float analogGetCmd (uint32_t servoIdx)
{
    if (servoIdx >= SERVO_COUNT) 
      return 0.5f;
    
    return ((float) av[servoIdx]/4096.0f);
}

void  analogCalibrateCurrentIntensity (void)
{
  // ?? Foam resitivity calibration ??
  for (uint32_t i=0; i<SERVO_COUNT+1; i++) {
    currentCalibrateValue[i] = i2cAvCurrent[i];
  }
}


float analogGetCurrentIntensity (uint32_t servoIdx)
{
  if (servoIdx >= (SERVO_COUNT+1)) 
      return 0xffffffff;

   const float rawVal =  i2cAvCurrent[servoIdx] - currentCalibrateValue[servoIdx];

    //    DebugTrace ("IDX[%d] currentCalibrateValue=%d rawVal=%d", servoIdx, currentCalibrateValue[servoIdx],
    //		av[adcI2cIndex[servoIdx]]);

    const float voltage = rawVal*VCC_VOLTAGE;
    
    //    DebugTrace ("Raw Vdiff = %.3f", voltage);
    const float current =  (voltage/0.185f);
    
    return fabs (current);
}


float analogGetRawPos (uint32_t servoIdx)
{
  if (servoIdx >= (SERVO_COUNT)) // there is 5 servos and 3.3v reference
    return 0xffffffff;		   


  return i2cAvPos[servoIdx];
}

// deduce regulated 5 volts power voltage from the measure of a 3.3v 
// precision voltage source. Should always stay around 5V otherwise
// it would be that 5V power source is not able to feed servos
float analogGet5VoltPowerVoltage (void)
{
  const float vref= 3.3f;

  return vref / i2cAvPos[SERVO_COUNT];
}



/*
 * Analog I/O thread
 */
msg_t thdAnalogIO(void *arg) 
{
  (void)arg;
  uint32_t timeStamp=0;
  float i2cAdcPos[I2C_MAX_CHANNELS]; // 5 servos + resistive foam in the clamp
  float i2cAdcCurrent[I2C_MAX_CHANNELS]; // 5 servo consumption
  bool_t i2cOk = TRUE;
  uint32_t lowPassIteration=0;

   /*
   * initialize the Analog IO (5 pins)
   */
  initAnalogIO ();
  chRegSetThreadName("thdAnalogIO");

  /* init i2c part */
  
  if  (initI2cDriver (&i2c2) != TRUE) {
    DebugTrace ("I2C (i2c2) init FAILED");
    syslog (LOG_FATAL, "I2C (i2c2) FAIL");
    i2cOk = FALSE;
  }
  
  while (!chThdShouldTerminate()) {
    chSysLock ();
    adcStopConversionI (&ADCD3);
    adcStartConversionI (&ADCD3, &adcgrpcfg3, samples3, ADC_BUF_DEPTH);
    chSysUnlock ();
    
    
    if (i2cOk && ( isActive(Analog_inCurrent) || isActive(Analog_inServiPos))) {
      static float medianFilterBuffer[ADC_MEDIAN_SIZE] [SERVO_COUNT+1] [MEDIAN_SIZE] =  {{{0.0f}}};
      // POSITION
      if (i2cGetADC_ADS7828_Val (i2c2.driver, I2C_ADC_SERVOPOS_ADR_OFFSET, 0b01011111, i2cAdcPos)) {
	DebugTrace ("i2cGetADC_ADS7828_Val error for position sensor");
	syslog (LOG_ERROR, "ADS7828 POS FAIL");
      } else {
	// first of all, the median filter
	static bool firstTime=TRUE;
	if (firstTime) {
	  firstTime = FALSE;
	  for (uint32_t i=0; i<SERVO_COUNT+1; i++) {
	    i2cAvPos[i] = i2cAdcPos[i];
	  }  
	} else {
	  for (uint32_t i=0; i<SERVO_COUNT+1; i++) {
	    // shift array and discard oldest value
	    memmove (&medianFilterBuffer[ADC_SERVO_POS][i][0],
		     &medianFilterBuffer[ADC_SERVO_POS][i][1],
		     sizeof(float) * MEDIAN_SIZE-1);

	    // copy new incomming value
	    medianFilterBuffer[ADC_SERVO_POS][i][MEDIAN_SIZE-1] = i2cAdcPos[i];

	    // copy to sort buffer
	    memcpy (&medianFilterBuffer[ADC_VALUE_SORT][i][0],
		     &medianFilterBuffer[ADC_SERVO_POS][i][0],
		     sizeof(float) * MEDIAN_SIZE);
	     
	    // sort the array
	    qsort (&medianFilterBuffer[ADC_VALUE_SORT][i][0], MEDIAN_SIZE, sizeof(float), 
		   &qsortCompareFloatCb);

	    // eliminate lowest and highest values
	    const float medianMean = (medianFilterBuffer[ADC_VALUE_SORT][i][1] + 
	      medianFilterBuffer[ADC_VALUE_SORT][i][2] + medianFilterBuffer[ADC_VALUE_SORT][i][3])/3.f;
	    
	    // then low pass filter
	    lowPassIteration++;
	    i2cAvPos[i] =  (( i2cAvPos[i]*(float)LOW_PASS_N) + medianMean)/ (float) (LOW_PASS_N+1);
	  }  
	}
      }
      
      // CURRENT
      if (i2cGetADC_ADS7828_Val (i2c2.driver, I2C_ADC_CURRENT_ADR_OFFSET, 0b00111111, i2cAdcCurrent)) {
	DebugTrace ("i2cGetADC_ADS7828_Val error for current sensor");
	syslog (LOG_ERROR, "ADS7828 CURRENT FAIL");
      } else {
	//for (uint32_t j=0; j<SERVO_COUNT+1; j++) {
	  //DebugTrace ("RC[%d]=%d", j, (int) (i2cAdcCurrent[j]*4096.0f));
	//}  

	static bool firstTime=TRUE;
	if (firstTime) {
	  firstTime = FALSE;
	  for (uint32_t i=0; i<SERVO_COUNT+1; i++) {
	    i2cAvCurrent[i] = i2cAdcCurrent[i];
	  }  
	} else {
	  for (uint32_t i=0; i<SERVO_COUNT+1; i++) {
	    i2cAvCurrent[i] =  ((i2cAvCurrent[i]*(float)LOW_PASS_N) + 
				i2cAdcCurrent[i])/ (float) (LOW_PASS_N+1);
	  }  
	}
      }
     
    }

    // check on power supply voltage
    if (lowPassIteration >= 100) { // wait for filter to stabilize
      if (analogGet5VoltPowerVoltage() < MIN_5VOLTS_POWER_SUPPLY_VOLTAGE) {
	if (isServoEngaged (SERVO_ALL_SERVOS)) {
	  // have to test validity of analogGet5VoltPowerVoltage() before using it
	  //	  servoDisengage (SERVO_ALL_SERVOS);
#pragma message \
  "enable  servoDisengage (SERVO_ALL_SERVOS) in " __FILE__	\
  " after testing validity of analogGet5VoltPowerVoltage()"
	  syslog (LOG_ERROR, "Surcharge Alim");
 	  syslog (LOG_ERROR, "Servos ** OFF **");	  
	  syslog (LOG_INFO, "rearmer : Bouton VERT");
	}
      }
    }
      //   check on  current intensity
    for (uint32_t i=0; i<SERVO_COUNT; i++) {
      if (i2cAvCurrent[i] >  max_current_intensity_use[i]) {
	if (isServoEngaged (i)) {
	  servoDisengage (i);
	  syslog (LOG_ERROR, "Surcharge Servo %d", i);
 	  syslog (LOG_ERROR, "Servos %d** OFF **", i);
	  syslog (LOG_INFO, "rearmer : Bouton VERT");
	}
      }
    }
  
    if (isActive(Analog_inCommand)) {
      if ((chTimeNow() - timeStamp) >= TIME_STEP) {
	timeStamp = chTimeNow();
	for (uint32_t i=0; i<SERVO_COUNT ; i++) {
	  const float adcV = analogGetCmd(i);
	  servoSetPos (i, adcV); 
	  /*   if (chTimeNow() > 5000) */
	  /*     chprintf (chp, "Apos=%.4f\r\n", adcV); */
	} 
      }
    } else {
      chThdSleepMilliseconds(1);
    }
  }
  
  closeAnalogIO();
  i2cStop (i2c2.driver);
  return 0;
}


static int qsortCompareFloatCb (const void *a, const void *b)
{
  const float va =  *((float *) a);
  const float vb =  *((float *) b);

  // positive if va > vb
  return (va >= vb) ? 1 : -1;
}
