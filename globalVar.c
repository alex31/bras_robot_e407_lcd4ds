#include "globalVar.h"

/*===========================================================================*/
/* USB related stuff.                                                        */
/*===========================================================================*/

/*
 * USB Driver structure.
 */

SerialUSBDriver SDU1;
BaseSequentialStream *chp = (BaseSequentialStream *) &SDU1;
bool_t jumperDebugPresent=TRUE;
MemoryHeap ccmHeap;
const uint8_t *UniqProcessorId = (uint8_t *) 0x1FFF7A10;
const uint8_t UniqProcessorIdLen = 12;

/* should use a timer to measure exact timing */
const uint32_t LsiFreq = 32768;

/*
exemple of uinq id

[1C] [0] [26] [0] [14] [47] [31] [31] [33] [35] [34] [31]
[38] [0] [3A] [0] [15] [47] [31] [31] [33] [35] [34] [31] 
[28] [0] [34] [0] [13] [47] [31] [31] [33] [35] [34] [31]
[29] [0] [33] [0] [17] [47] [31] [31] [33] [35] [34] [31] 
[1E] [0] [3A] [0] [0B] [47] [31] [33] [31] [33] [30] [37] 
 */

/*===========================================================================*/
/* Heap in ccm for malloc stuff */
/*===========================================================================*/
uint8_t ccmHeapBuffer[32*1024] __attribute__ ((section(".ccmram"), aligned(8))) ;
