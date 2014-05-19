#include "jbus485.h"
#include "jbus_common.h"
#include "ch.h"
#include "hal.h"
#include "globalVar.h"
#include "stdutil.h"
#include "servo_pwm.h"
#include "modbus/port/port.h"
#include "modbus/include/mb.h"
#include "modbus/include/mbport.h"
#include "modbus/include/mbutils.h"

/* ----------------------- Static Function Declaration ------------------------------------------*/


/* ----------------------- Defines ------------------------------------------*/


/* ----------------------- Static variables ---------------------------------*/
typedef struct {
  uint32_t	baudRate;
  eMBMode	mode;
  eMBParity	parity;
  uint8_t	nbBitsStop;
  uint8_t	address;
  int8_t	regOffset;
} Jbus485State;



static Jbus485State __attribute__((aligned(8)))
jbus485States = {
  .baudRate = 9600,
  .mode = MB_RTU,
  .parity = MB_PAR_NONE,
  .nbBitsStop = 1,
  .address = 10,
  .regOffset=0,
};



/*
  jbus baud 38400
  jbus nopar
  jbus stopbit 1
  jbus proto rtu
  jbus addr 10
  jbus save
 */

// === PROTECTED HOLDING ===
// test with modpoll -m rtu -a 10 -r 1666 -c 20 -t 4 -b 38400 -d 8 -s 1 -p none /dev/ttyUSB0
// === REG HOLDING ===
// test with modpoll -m rtu -a 10 -r 100 -c 19 -t 4 -b 38400 -d 8 -s 1 -p none /dev/ttyUSB0
// === REG INPUT ===
// test with modpoll -m rtu -a 10 -r 200 -c 31 -t 3 -b 38400 -d 8 -s 1 -p none /dev/ttyUSB0
// === COIL ===
// modpoll -m rtu -a 10 -r 300 -c 10 -t 0 -b 38400 -d 8 -s 1 -p none /dev/ttyUSB0
// === DISCRETE ===
// modpoll -m rtu -a 10 -r 400 -c 25 -t 1 -b 38400 -d 8 -s 1 -p none /dev/ttyUSB0

/* ----------------------- Start implementation -----------------------------*/




bool_t initJbus485 (void)
{
  eMBErrorCode    eStatus;
  getJbus485StatesFromEeprom ();

  if ((eStatus = eMBInit (jbus485States.mode, jbus485States.address, 
			  1, jbus485States.baudRate, 
			  jbus485States.parity, jbus485States.nbBitsStop)) != MB_ENOERR) {
    DebugTrace ("eMBInit Failed err=%d", eStatus);
    return FALSE;
  }
  
  if ((eStatus = eMBSetSlaveID (0x42, TRUE, UniqProcessorId, UniqProcessorIdLen))  != MB_ENOERR) {
    DebugTrace ("eMBSetSlaveID Failed err=%d", eStatus);
    return FALSE;
  }
  
  if ((eStatus = eMBEnable())  != MB_ENOERR) {
    DebugTrace ("eMBEnable Failed err=%d", eStatus);
    return FALSE;
  } 
  pxMBPortCBTimerExpired ();
  
  return TRUE;
}


/*
 * logical level switch I/O thread
 */
msg_t thdJbus485IO(void *arg) 
{
  (void)arg;

  chRegSetThreadName("thd JBUS 485 IO");
  while  (initJbus485 () != TRUE) {
    chThdSleepMilliseconds(1000);
    if (chThdShouldTerminate()) 
      goto cleanAndExit;
  }

  chThdSleepMilliseconds(10);

    do {
      eMBPoll ();
    }  while (!chThdShouldTerminate());

    
 cleanAndExit:
  eMBDisable ();
  eMBClose ();
  DebugTrace ("thdJbusIO 485 Thread Is stopping");

  return 0;
}



void setBaudRate (uint32_t baudRate) 
{
  jbus485States.baudRate = baudRate;
}

void setJbusSerialMode (eMBMode	mode) 
{
  jbus485States.mode = mode;
}

void setParity (eMBParity parity) 
{
  jbus485States.parity = parity;
}

void setNbBitsStop ( uint8_t	nbBitsStop) 
{
  jbus485States.nbBitsStop = nbBitsStop;
}

void setMbAddressFor485 (uint8_t address) 
{
  jbus485States.address = address;
}

void setRegOffset (int8_t offset)
{
  jbus485States.regOffset = offset;
}

uint32_t      getBaudRate (void) 
{
  return jbus485States.baudRate ;
}

eMBMode       getJbusSerialMode (void) 
{
  return jbus485States.mode ;
}

eMBParity     getParity (void) 
{
  return jbus485States.parity ;
}

uint8_t	      getNbBitsStop (void) 
{
  return jbus485States.nbBitsStop ;
}

uint8_t	      getMbAddressFor485 (void) 
{
  return jbus485States.address ;
}
int8_t	      getRegOffset (void)
{
  return jbus485States.regOffset ;
}

ErrorCond     getJbus485StatesFromEeprom (void)
{
  return eepromLoad (EEPROM_JBUS_SERIAL, &jbus485States, sizeof(jbus485States));
}


ErrorCond     storeJbus485StatesToEeprom (void)
{
  return eepromStore (EEPROM_JBUS_SERIAL, &jbus485States, sizeof(jbus485States));
}
