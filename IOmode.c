#include "ch.h"
#include "printf.h"
#include "globalVar.h"
#include "IOmode.h"
#include "logicLevelIO.h"
#include "analogicIO.h"
#include "jbus485.h"
#include "jbusEther.h"
#include "stdutil.h"
#include "lcdDisplay.h"
#include "userInput.h"



static enum IO_modes ioMode = IO_mode_Off;
static enum IO_bits  ioBits = Logic_none;

struct IO_Threads {
  enum IOThreads_bits currentThreads;
  Thread *chLogicThd;
  Thread *chAnalogThd;
  Thread *chJbus485Thd;
  Thread *chJbusEtherThd;
} iOThreads = {
  .currentThreads = 0,
  .chLogicThd = NULL,
  .chAnalogThd = NULL,
  .chJbus485Thd = NULL,
  .chJbusEtherThd = NULL
};


bool_t isActive (enum IO_bits bit)
{
  //  return (getIoBits(ioMode) & bit);
  return (ioBits & bit);
}

void setIoBit (enum IO_bits bit)
{
  ioBits |= bit;
}

void clearIoBit (enum IO_bits bit)
{
  ioBits &= ~bit;
}

enum IO_bits getIoBits (enum IO_modes mode)
{
  enum IO_bits retVal=0;

  switch (mode) {
  case IO_mode_Off:
  case Mode_Tuning:
    retVal = Logic_out|Analog_inCurrent|Analog_inServiPos; 
    break;

  case Mode_Logic:
    retVal = Logic_in|Logic_out|Analog_inCurrent|Analog_inServiPos|ServoPwm_out; 
    break;

  case Mode_Analog_Pos:
    retVal = Analog_inCommand|Analog_inCurrent|Analog_inServiPos|Logic_out|ServoPwm_out;
    break;

  case Mode_Analog_Calib:
    retVal = Analog_inCurrent|Analog_inServiPos|Logic_out;
    break;

  case Mode_Jbus485:
    retVal = Jbus485_inout|Analog_inCurrent|Analog_inServiPos|Logic_out|ServoPwm_out;
    break;

  case Mode_JbusEther:
    retVal = JbusEther_inout|Analog_inCurrent|Analog_inServiPos|Logic_out|ServoPwm_out;
    break;

  case IO_mode_End:
  default:
    retVal = 0;
    break;
  }
  return retVal;
}

enum IOThreads_bits getIoThreads (enum IO_bits iobit)
{
  enum IOThreads_bits retVal=0;

  if (iobit & Logic_in)
    retVal |= THD_Logic;

  if (iobit & Logic_out)
    retVal |= THD_Logic;

  if (iobit & Analog_inCommand)
    retVal |= THD_Analog;

  if (iobit & Analog_inCurrent)
    retVal |= THD_Analog;

  if (iobit & Analog_inServiPos)
    retVal |= THD_Analog;

  if (iobit & Jbus485_inout)
    retVal |= THD_Jbus485;

  if (iobit & JbusEther_inout)
    retVal |= THD_JbusEther;
  
  return retVal;
}

void setMode (enum IO_modes mode)
{
  const enum IO_bits nextIoBits = getIoBits (mode);
  
  
  static const char *modeName[] = {
    "IO_mode_Off", 
    "Mode_Logic", "Mode_Analog_Pos", "Mode_Analog_Calib", "Mode_Jbus485", "Mode_JbusEther", 
    "Mode_Tuning", "IO_mode_End"
  };
  

  if (mode != ioMode) {
    // if we quit ethernet mode, we have to reset
    // because i don't know how to reinitialise
    // freemodbus and lwip states
    bool_t hasToReset = (ioMode == Mode_JbusEther);
    ioMode = mode;
    DebugTrace ("input mode %s", modeName[mode]);
    syslog (LOG_INFO, "mode %s", modeName[mode]);
    
    if (hasToReset) {
      //      systemReset ();
    }
    
    if (isArmBoardPlugged() || jumperDebugPresent) {
      // for any mode change, we park the arm
      // and wait for park to complete
      for (uint32_t i = 0; i< SERVO_COUNT; i++) {
	servoSetTopSpeed (i,3);
	servoPark (i);
      }
      
      if (isActive (ServoPwm_out)) {
      syslog (LOG_INFO, "parc du bras");
      // wait for park to complete before changing mode
	for (uint32_t i = 0; i< SERVO_COUNT; i++) {
	  while (isServoMoving (i)) {
	    chThdSleepMilliseconds(10);
	  }
	  syslog (LOG_INFO, "bras[%d] ok", i);
	}
      }
      
      // restore init speed
      getServoStatesFromEeprom ();
      manageThreadAccordingToMode (ioBits, nextIoBits);
      ioBits = nextIoBits;
    }
  }

}


enum IO_modes  getMode (void)
{
  return ioMode;
}


static WORKING_AREA(waThdLogicLevelIO, CHPRINT_WA_SIZE);
static WORKING_AREA(waThdAnalogIO, CHPRINT_WA_SIZE);
static WORKING_AREA(waThdJBusIO, 8*1024); // 8 ko stack for lwip/ethernet stuff

void manageThreadAccordingToMode (enum  IO_bits oldIoBits, enum IO_bits nextIoBits)
{
  enum IOThreads_bits oldThreads = getIoThreads (oldIoBits);
  enum IOThreads_bits newThreads = getIoThreads (nextIoBits);

  enum IOThreads_bits threadToKill = oldThreads & (~newThreads);
  enum IOThreads_bits threadToLaunch = newThreads & (~oldThreads);

  if ((threadToKill & THD_Logic) && (iOThreads.chLogicThd != NULL)) {
    DebugTrace ("	STOP LOGIC thread");
    chThdTerminate (iOThreads.chLogicThd); 
    chThdWait (iOThreads.chLogicThd);
    // chThdRelease (iOThreads.chLogicThd);
    iOThreads.chLogicThd = NULL;
  }

  if ((threadToKill & THD_Analog) && (iOThreads.chAnalogThd != NULL)) {
    DebugTrace ("	STOP ANALOG thread");
    chThdTerminate (iOThreads.chAnalogThd);
    chThdWait (iOThreads.chAnalogThd);
    // chThdRelease (iOThreads.chAnalogThd);
    iOThreads.chAnalogThd = NULL;
  }
  
  if ((threadToKill & THD_Jbus485) && (iOThreads.chJbus485Thd != NULL)) {
    DebugTrace ("	STOP JBUS 485 thread");
    chThdTerminate (iOThreads.chJbus485Thd);
    chThdWait (iOThreads.chJbus485Thd);
    // chThdRelease (iOThreads.chJbus485Thd);
    iOThreads.chJbus485Thd = NULL;
  }
  
  if ((threadToKill & THD_JbusEther) && (iOThreads.chJbusEtherThd != NULL)) {
    DebugTrace ("	STOP JBUS ETHERNET thread");
    chThdTerminate (iOThreads.chJbusEtherThd);
    chThdWait (iOThreads.chJbusEtherThd);
    // chThdRelease (iOThreads.chJbusEtherThd);
    iOThreads.chJbusEtherThd = NULL;
  }
  
  /* DebugTrace ("oldThreads=0x%x newThreads=0x%x threadToKill=0x%x threadToLaunch=0x%x", */
  /* 	    oldThreads, newThreads, threadToKill, threadToLaunch); */
  /* chThdSleepMilliseconds(100);*/


  if ((threadToLaunch & THD_Logic) && (iOThreads.chLogicThd == NULL)) {
  /*
   * Creates the logic io thread.
   */
    DebugTrace ("	launch LOGIC thread");
    iOThreads.chLogicThd =   chThdCreateStatic (waThdLogicLevelIO, sizeof(waThdLogicLevelIO), 
						NORMALPRIO, thdLogicLevelIO, NULL);
    
  }

  if ((threadToLaunch & THD_Analog) && (iOThreads.chAnalogThd == NULL)) {
  /*
   * Creates the Analog io thread.
   */
    DebugTrace ("	launch ANALOG thread");
    iOThreads.chAnalogThd =   chThdCreateStatic (waThdAnalogIO, sizeof(waThdAnalogIO), 
						 NORMALPRIO, thdAnalogIO, NULL);
    
  }


  if ((threadToLaunch & THD_Jbus485) && (iOThreads.chJbus485Thd == NULL)) {
  /*
   * Creates the Jbus over rs 485 io thread.
   */
    DebugTrace ("	launch JBUS 485 thread");
    iOThreads.chJbus485Thd = chThdCreateStatic (waThdJBusIO, sizeof(waThdJBusIO), 
						NORMALPRIO, thdJbus485IO, NULL);
    
  }

  if ((threadToLaunch & THD_JbusEther) && (iOThreads.chJbusEtherThd == NULL)) {
  /*
   * Creates the Jbus over rs Ether io thread.
   */
    DebugTrace ("	launch JBUS Ether thread");
    iOThreads.chJbusEtherThd = chThdCreateStatic (waThdJBusIO, sizeof(waThdJBusIO), 
						NORMALPRIO, thdJbusEtherIO, NULL);
    
  }
}


void IoModeInit (void)
{
  clearIoBit (ServoPwm_out);
  setMode (IO_mode_Off);
}

/* ErrorCond     getIoModeFromEeprom (void) */
/* {  */
/*   enum IO_modes mode=IO_mode_Off; */
  
/*   ErrorCond ec =  eepromLoad (EEPROM_IOMODE, &mode, sizeof(ioMode)); */
/*   setMode (mode); */

/*   return ec; */
/* } */

/* ErrorCond     storeIoModeToEeprom (void)  */
/* { */
/*   return eepromStore (EEPROM_IOMODE, &ioMode, sizeof(ioMode)); */
/* } */




