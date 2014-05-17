#ifndef __GLOBAL_VAR_H__
#define __GLOBAL_VAR_H__

#include "ch.h"
#include "hal.h"

#include "usb_serial.h"


/*===========================================================================*/
/* uniq id of stm32 processor                                                        */
/*===========================================================================*/

extern const uint8_t *UniqProcessorId ;
extern const uint8_t UniqProcessorIdLen ;

  /* should use a timer to measure exact timing */
extern  const uint32_t LsiFreq;
extern bool_t jumperDebugPresent;


/*===========================================================================*/
/* USB related stuff.                                                        */
/*===========================================================================*/

/*
 * USB Driver structure.
 */


extern SerialUSBDriver SDU1;
extern BaseSequentialStream *chp;
extern MemoryHeap ccmHeap;
extern uint8_t ccmHeapBuffer[32*1024];
#endif //  __GLOBAL_VAR_H__
