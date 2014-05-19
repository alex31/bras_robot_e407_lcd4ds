#include "jbus_common.h"
#include "jbus485.h"
#include "ch.h"
#include "hal.h"
#include "printf.h"
#include "globalVar.h"
#include "stdutil.h"
#include "servo_pwm.h"
#include "rtcAccess.h"
#include "lcdDisplay.h"
#include "analogicIO.h"
#include "calibration.h"
#include "modbus/port/port.h"
#include "modbus/include/mb.h"
#include "modbus/include/mbport.h"
#include "modbus/include/mbutils.h"
#include "jbus485.h"
#include <string.h>

/* ----------------------- Static Function Declaration ------------------------------------------*/
static bool_t isItValidAdress (uint32_t addr, uint32_t base, uint32_t len);
static bool_t isInInterval_f (float val, float min, float max);
static bool_t isInInterval_i (int32_t val, int32_t min, int32_t max);
static void storeReg ( uint8_t** pucReg, uint16_t val);
static uint16_t loadReg ( uint8_t** pucReg);
static bool_t getOrSetRegHoldingValue (uint16_t iRegIndex,  uint8_t **curPucReg, eMBRegisterMode eMode);
static bool_t getOrSetProtectedRegHoldingValue (uint16_t iRegIndex,  uint8_t **curPucReg, 
						eMBRegisterMode eMode);
static bool_t getOrSetCoilValue (uint16_t iRegIndex,  uint8_t *curPucReg, eMBRegisterMode eMode);
static bool_t getRegInputValue (uint16_t iRegIndex, uint8_t **curPucReg);
static bool_t getDiscreteValue (uint16_t iRegIndex,  uint8_t *curPucReg);
static uint16_t normalisedToReg (float val); 
static float regToNormalised (uint16_t val); 
static uint16_t floatToRegFact1000 (float val); 
static float regToFloatFact1000 (uint16_t val); 
static void setLastFunction (ModbusFunction f);
static void setLastAction (eMBRegisterMode m);
static void setLastAddress (uint16_t a);
static void setLastNbReg  (uint16_t n);
static void setLastValue  (uint16_t n);

/* ----------------------- Defines ------------------------------------------*/

#define NORMALISED_TO_REG_SCALE (1000.0f) // valeur de multiplication des valeur flottante
// normalisée (0 <= 1) vers des registres unsigned short dialogue automate 

/* ----------------------- Static variables ---------------------------------*/
const  uint32_t regHoldingBase= 100;
const  uint32_t regInputBase= 200;
const  uint32_t regCoilBase= 300;
const  uint32_t regDiscreteBase= 400;
const  uint32_t protectedRegHoldingBase= 1666;

typedef enum  {LCD1=0, LCD2, JOUR, MOIS, ANNEE, JOUR_SEMAINE, HEURE, MINUTE, SECONDE, 
	       CONSIGNE, VITESSE_MAX, REG_HOLDING_END} RegHolding;
typedef enum  {POS_COURANTE=0, VITESSE_COURANTE, POSITION_PARK, POS_POTAR_SERVO_RAW,
	       POS_POTAR_SERVO_HOMOGENE, INTENSITE_SERVO, REG_INPUT_END} RegInput;
typedef enum  {SETPARK=0, GOTO_PARK, REG_COIL_END} RegCoil;
typedef enum  {MOVING=0, JAMED, ENABLED, BUTEE_MIN, BUTEE_MAX, REG_DISCRETE_END} RegDiscrete;
typedef enum  {PASSWD1, PASSWD2, ADRJBUS, BAUD, NBBITSSTOP,  PARITE,
	       ADDR_IP_1, ADDR_IP_2, NETMASK_1, NETMASK_2,  PWMMIN, PWMMAX, 
	       PROTECTED_REG_HOLDING_END} ProtectedRegHolding;

typedef enum {PROTECTED, HALF_PROTECTED, UNPROTECTED} ProtectedState;


static ProtectedState protectedState = PROTECTED;
static ModbusFunction modbusFunction = FUNC_NONE;
static eMBRegisterMode modbusAction = MB_REG_READ;
static uint16_t lastAddress = 0;
static uint16_t lastNbRegs = 0;
static uint16_t lastValue = 0;
static eMBErrorCode lastStatus;

static char l1[30], l2[30], l3[30];
/* ----------------------- Start implementation -----------------------------*/
static bool_t isItValidAdress (uint32_t addr, uint32_t base, uint32_t len)
{
  return ((addr >= base) && (addr <= (base+len)));
}

static bool_t isInInterval_f (float val, float min, float max)
{
  return ((val >= min) && (val <= max)); 
}

static bool_t isInInterval_i (int32_t val, int32_t min, int32_t max)
{
  return ((val >= min) && (val <= max)); 
}

static void storeReg ( uint8_t** pucReg, uint16_t val)
{
  //  DebugTrace ("addr =0x%x v=%d", *pucReg, val);
  setLastValue(val);
  **pucReg = (uint8_t) (val >> 8);
  (*pucReg)++;
  **pucReg = (uint8_t) (val & 0xFF) ;
  (*pucReg)++;
}

static uint16_t  loadReg ( uint8_t** pucReg)
{
  uint16_t val = **pucReg << 8;
  (*pucReg)++;
  val |= **pucReg;
  (*pucReg)++;
  setLastValue(val);
  return val;
}






eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
  eMBErrorCode    eStatus = MB_ENOERR;
  uint32_t        iRegIndex;
  uint8_t *curPucReg = pucRegBuffer;
  const uint32_t protectedRegHoldingEnd =  PROTECTED_REG_HOLDING_END + 
    ((PROTECTED_REG_HOLDING_END - PWMMIN) * SERVO_COUNT);
  const uint32_t regHoldingEnd =  REG_HOLDING_END + 
    ((REG_HOLDING_END - CONSIGNE) * SERVO_COUNT);

  usAddress += getRegOffset(); // in case of offset range between plc and freemobus 

  //  DebugTrace ("HoldingCB ad[0x%x] usNRegs[%d] eMode[%d]", usAddress, usNRegs, eMode);
  setLastFunction (FUNC_HOLDING);
  setLastAction (eMode);
  setLastAddress (usAddress);
  setLastNbReg (usNRegs);
  
  if (isItValidAdress (usAddress, regHoldingBase, regHoldingEnd) &&
      isItValidAdress (usAddress+usNRegs, regHoldingBase, regHoldingEnd)) {
    iRegIndex = (uint32_t) usAddress - regHoldingBase;
    while (usNRegs > 0) {
      if (getOrSetRegHoldingValue (iRegIndex, &curPucReg, eMode) == FALSE) {
	eStatus = MB_EINVAL;
	goto cleanAndExitHCB;
      }
      iRegIndex++;
      usNRegs--;
    }

  } else if (isItValidAdress (usAddress, protectedRegHoldingBase, protectedRegHoldingEnd) &&
      isItValidAdress (usAddress+usNRegs,protectedRegHoldingBase, protectedRegHoldingEnd)) {
    iRegIndex = (uint32_t) usAddress - protectedRegHoldingBase;
    while (usNRegs > 0) {
      if (getOrSetProtectedRegHoldingValue (iRegIndex, &curPucReg, eMode) == FALSE) {
	eStatus = MB_EINVAL;
	goto cleanAndExitHCB;
      }
      iRegIndex++;
      usNRegs--;
    }
    
  } else {
    eStatus = MB_ENOREG;
    DebugTrace ("Register bad @ %d/%d",  usAddress, usNRegs);
    syslog (LOG_ERROR, "Register bad @ %d/%d",  usAddress, usNRegs);
  }
  

 cleanAndExitHCB:
  return (lastStatus = eStatus);
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
  eMBErrorCode    eStatus = MB_ENOERR;
  uint32_t        iRegIndex;
  uint8_t *curPucReg = pucRegBuffer;
  usAddress += getRegOffset(); // in case of offset range between plc and freemobus 

  //DebugTrace ("InputCB ad[0x%x] usNRegs[%d]", usAddress, usNRegs);
  setLastFunction (FUNC_INPUT);
  setLastAddress (usAddress);
  setLastNbReg (usNRegs);

  if (isItValidAdress (usAddress, regInputBase, (REG_INPUT_END*SERVO_COUNT)+1) &&
      isItValidAdress (usAddress+usNRegs, regInputBase, (REG_INPUT_END*SERVO_COUNT)+1)) {
    iRegIndex = (uint32_t) usAddress - regInputBase;
    while (usNRegs > 0) {
      if (getRegInputValue (iRegIndex, &curPucReg) == FALSE) {
	eStatus = MB_EINVAL;
	goto cleanAndExitICB;
      }
      iRegIndex++;
      usNRegs--;
    }
  } else {
    eStatus = MB_ENOREG;
    syslog (LOG_ERROR, "Input bad @ %d/%d",  usAddress, usNRegs);
  }
  
 cleanAndExitICB:
  return (lastStatus = eStatus);
}

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
  eMBErrorCode    eStatus = MB_ENOERR;
  uint32_t        iRegIndex;
  usAddress += getRegOffset(); // in case of offset range between plc and freemobus 

  //DebugTrace ("RegCoilsCB ad[0x%x] usNCoils[%d]", usAddress, usNCoils);
  setLastFunction (FUNC_COIL);
  setLastAction (eMode);
  setLastAddress (usAddress);
  setLastNbReg (usNCoils);


  if (isItValidAdress (usAddress, regCoilBase, REG_COIL_END*SERVO_COUNT) &&
      isItValidAdress (usAddress+usNCoils, regCoilBase, REG_COIL_END*SERVO_COUNT)) {
    iRegIndex = (uint32_t) usAddress - regCoilBase;
    while (usNCoils > 0) {
      if (getOrSetCoilValue (iRegIndex, pucRegBuffer, eMode) == FALSE) {
	eStatus = MB_EINVAL;
	goto cleanAndExitICB;
      }
      iRegIndex++;
      usNCoils--;
    }
  } else {
    eStatus = MB_ENOREG;    
    syslog (LOG_ERROR, "Coil bad @ %d/%d",  usAddress, usNCoils);
  }
  
 cleanAndExitICB:
  return (lastStatus = eStatus);
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
  eMBErrorCode    eStatus = MB_ENOERR;
  uint32_t        iRegIndex;
  usAddress += getRegOffset(); // in case of offset range between plc and freemobus 

  //DebugTrace ("RegDiscreteCB ad[0x%x] usNDiscrete[%d]", usAddress, usNDiscrete);
  setLastFunction (FUNC_DISCRETE);
  setLastAddress (usAddress);
  setLastNbReg (usNDiscrete);

  if (isItValidAdress (usAddress, regDiscreteBase, REG_DISCRETE_END*SERVO_COUNT) &&
      isItValidAdress (usAddress+usNDiscrete, regDiscreteBase, REG_DISCRETE_END*SERVO_COUNT)) {
    iRegIndex = (uint32_t) usAddress - regDiscreteBase;
    while (usNDiscrete > 0) {
      if (getDiscreteValue (iRegIndex, pucRegBuffer) == FALSE) {
	eStatus = MB_EINVAL;
	goto cleanAndExitDCB;
      }
      iRegIndex++;
      usNDiscrete--;
    }
  } else {
    eStatus = MB_ENOREG;
    syslog (LOG_ERROR, "Discrete bad @ %d/%d",  usAddress, usNDiscrete);
  }
  
  
 cleanAndExitDCB:
  return (lastStatus = eStatus);
}

static uint16_t normalisedToReg (float val)
{
  return (( uint16_t) (val * NORMALISED_TO_REG_SCALE));
}

static float regToNormalised (uint16_t val)
{
  return ((float) val / NORMALISED_TO_REG_SCALE);
}

static uint16_t floatToRegFact1000 (float val)
{
  return (( uint16_t) (val * 1000.0f));
}


static float regToFloatFact1000 (uint16_t val)
{
  return ((float) val / 1000.0f);
}



// JOUR, MOIS, ANNEE,  JOUR_SEMAINE, HEURE, MINUTE, SECONDE, CONSIGNE (x 5), VITESSE_MAX (x 5)
 bool_t getOrSetRegHoldingValue (uint16_t iRegIndex,  uint8_t **curPucReg, 
				       eMBRegisterMode eMode)
{
  /* RegHolding type = iRegIndex % REG_HOLDING_END; */
  /* uint32_t servoNum = iRegIndex / REG_HOLDING_END; */
  /* float loadVal; */

  RegHolding type = iRegIndex ;
  uint32_t servoNum = -1;
  float loadVal;
  uint32_t rtcVal;


  // first register are uniq, but after (and including) CONSIGNE, it's SERVO_NUM tupples of CONSIGNE,VITESSE_MAX
  if (iRegIndex >= CONSIGNE) {
    type = CONSIGNE + ((iRegIndex-CONSIGNE) % (REG_HOLDING_END-CONSIGNE));
    servoNum = (iRegIndex-CONSIGNE) / (REG_HOLDING_END-CONSIGNE);
    
    if (servoNum >= SERVO_COUNT) {
      syslog (LOG_WARN, "servo>5 : %d", servoNum);
      DebugTrace ("getOrSetRegHoldingValue servo>5 : %d", servoNum);
      return FALSE;
    }
  } 
    
  //DebugTrace ("iRegIndex=%d type=%d servoNum=%d", iRegIndex, type, servoNum);
  
  switch (type) {
  case LCD1:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getLcd1Register()); 
    } else {
      setLcd1Register (loadReg (curPucReg));
    }
    break;

  case LCD2:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getLcd2Register()); 
    } else {
      setLcd2Register (loadReg (curPucReg));
    }
    break;

  case JOUR:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getMonthDay());
    } else {
      rtcVal = loadReg (curPucReg);
      if (isInInterval_i (rtcVal, 0, 31)) {
	setMonthDay (rtcVal);
      } else {
	DebugTrace ("JOUR, val %d is not in range 0-31", rtcVal);
	syslog (LOG_WARN, "jour[0-31] : %d", rtcVal);
	return FALSE;
      }
    }
    break;

  case MOIS:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getMonth());
    } else {
      rtcVal = loadReg (curPucReg);
      if (isInInterval_i (rtcVal, 1, 12)) {
	setMonth (rtcVal);
      } else {
	DebugTrace ("MOIS, val %d is not in range 1-12", rtcVal);
	syslog (LOG_WARN, "mois[1-12] : %d", rtcVal);
	return FALSE;
      }
    } 
    break;

  case ANNEE:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getYear()+1900);
    } else {
      rtcVal = loadReg (curPucReg);
      if (isInInterval_i (rtcVal, 2000, 2100)) {
	 setYear (rtcVal-1900);
      } else {
	DebugTrace ("ANNEE, val %d is not in range 2000-2100", rtcVal);
	syslog (LOG_WARN, "ANNEE[2000-2100] : %d", rtcVal);
	return FALSE;
      }
    }
    break;

   case JOUR_SEMAINE:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getWeekDay());
    } else {
      rtcVal = loadReg (curPucReg);
      if (isInInterval_i (rtcVal, 1, 7)) {
	 setWeekDay (rtcVal);
      } else {
	DebugTrace ("JOUR_SEMAINE, val %d is not in range 1-7", rtcVal);
	syslog (LOG_WARN, "jour semaine [1-7] : %d", rtcVal);
	return FALSE;
      }
    }
    break;

  case HEURE:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getHour());
    } else {
      rtcVal = loadReg (curPucReg);
      if (isInInterval_i (rtcVal, 0, 23)) {
	setHour (rtcVal);
      } else {
	DebugTrace ("HEURE, val %d is not in range 0-23", rtcVal);
	syslog (LOG_WARN, "heure[0-23] : %d", rtcVal);
	return FALSE;
      }
    }
    break;

   case MINUTE:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getMinute());
    } else {
      rtcVal = loadReg (curPucReg);
      if (isInInterval_i (rtcVal, 0, 59)) {
	 setMinute (rtcVal);
      } else {
	DebugTrace ("MINUTE, val %d is not in range 0-59", rtcVal);
	syslog (LOG_WARN, "minute [0-59] : %d", rtcVal);
	return FALSE;
      }
    }
    break;

   case SECONDE:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getSecond());
    } else {
      rtcVal = loadReg (curPucReg);
      if (isInInterval_i (rtcVal, 0, 59)) {
	 setSecond (rtcVal);
      } else {
	DebugTrace ("SECONDE, val %d is not in range 0-59", rtcVal);
	syslog (LOG_WARN, "seconde [0-59] : %d", rtcVal);
	return FALSE;
      }
    }
    break;
    
  case CONSIGNE:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, normalisedToReg (servoGetPos (servoNum)));
    } else {
      loadVal = regToNormalised (loadReg (curPucReg));
      if (isInInterval_f (loadVal, 0.0f, 1.0f)) {
	servoSetPos (servoNum, loadVal);
      } else {
	DebugTrace ("CONSIGNE, val %f is not in range 0-1", loadVal);
	syslog (LOG_WARN, "position [0-1000] : %d", normalisedToReg(loadVal));
	return FALSE;
      }
    }
    break;

  case VITESSE_MAX:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, normalisedToReg (servoGetTopSpeed (servoNum)));
    } else {
      loadVal = regToNormalised (loadReg (curPucReg));
      if (isInInterval_f (loadVal, 0.0f, 60.0f)) {
	servoSetTopSpeed (servoNum, loadVal);
      } else {
	DebugTrace ("VITESSE_MAX, val %f is not in range 0-1000", loadVal);
	syslog (LOG_WARN, "vitesse max [0-60000] : %d", normalisedToReg(loadVal));
	return FALSE;
      }
    }    
    break;

 
  case REG_HOLDING_END:
    break;
  }

  return TRUE;
}


static bool_t getOrSetProtectedRegHoldingValue (uint16_t iRegIndex,  uint8_t **curPucReg, 
						eMBRegisterMode eMode)
{
  ProtectedRegHolding type = iRegIndex;
  int32_t servoNum=-1;
  static int32_t ipAddress=0, netmask=0;
  
  if (iRegIndex >= PWMMIN) {
    type = PWMMIN + ((iRegIndex-PWMMIN) % (PROTECTED_REG_HOLDING_END-PWMMIN));
    servoNum = (iRegIndex-PWMMIN) / (PROTECTED_REG_HOLDING_END-PWMMIN);
  } 

  if (servoNum >= SERVO_COUNT) {
    syslog (LOG_WARN, "servo>5 : %d", servoNum);
    return FALSE;
  }

  switch (type) {
  case PASSWD1:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, 0xBADA); // on ne doit pas lire le mot de passe
    } else {
      if (loadReg (curPucReg) == 0xB16B) {
	protectedState = HALF_PROTECTED;
      } else {
	DebugTrace ("Bad Passwd1");
	syslog (LOG_WARN, "Bad Passwd1", servoNum);
	return FALSE;
      }
    }
    break;

  case PASSWD2:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, 0xCCE5); // on ne doit pas lire le mot de passe
    } else {
      if (loadReg (curPucReg) == 0x00B5) {
	protectedState = protectedState == HALF_PROTECTED ? UNPROTECTED : PROTECTED;
      } else {
	DebugTrace ("Bad Passwd2");
	syslog (LOG_WARN, "Bad Passwd2", servoNum);
	return FALSE;
      }
    }
    break;



  case ADRJBUS:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getMbAddressFor485());
    } else {
      if (protectedState != UNPROTECTED) {
	DebugTrace ("Protected Mode");
	syslog (LOG_WARN, "Addr jbus: Protected Mode", servoNum);
	return FALSE;
      } else {
	setMbAddressFor485 ((uint8_t) loadReg (curPucReg));
      }
    } 
    break;

  case BAUD:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getBaudRate());
    } else {
      if (protectedState != UNPROTECTED) {
	DebugTrace ("Protected Mode");
	syslog (LOG_WARN, "setBaud: Protected Mode", servoNum);
	return FALSE;
      } else {
	setBaudRate (loadReg (curPucReg));
      }
    } 
    break;
   

  case NBBITSSTOP:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getNbBitsStop());
    } else {
      if (protectedState != UNPROTECTED) {
	DebugTrace ("Protected Mode");
	syslog (LOG_WARN, "nb bit stop: Protected Mode", servoNum);
	return FALSE;
      } else {
	setNbBitsStop ((uint8_t) loadReg (curPucReg));
      }
    } 
    break;

  case PARITE:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, getParity());
    } else {
      if (protectedState != UNPROTECTED) {
	DebugTrace ("Protected Mode");
	syslog (LOG_WARN, "parite: Protected Mode", servoNum);
	return FALSE;
      } else {
	setParity (loadReg (curPucReg));
      }
    } 
    break;

  case ADDR_IP_1:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, 0xffff); // recuperer l'adresse ip
    } else {
      if (protectedState != UNPROTECTED) {
	DebugTrace ("Protected Mode");
	syslog (LOG_WARN, "addr ip/1: Protected Mode", servoNum);
	return FALSE;
      } else {
	ipAddress = loadReg (curPucReg) << 16;
      }
    } 
    break;

  case ADDR_IP_2:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, 0xffff); // recuperer l'adresse ip
    } else {
      if (protectedState != UNPROTECTED) {
	DebugTrace ("Protected Mode");
	syslog (LOG_WARN, "addr ip/2: Protected Mode", servoNum);
	return FALSE;
      } else {
	ipAddress |= loadReg (curPucReg);
      }
    } 
    break;

  case NETMASK_1:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, 0xffff); // recuperer l'adresse ip
    } else {
      if (protectedState != UNPROTECTED) {
	DebugTrace ("Protected Mode");
	syslog (LOG_WARN, "netmask1: Protected Mode", servoNum);
	return FALSE;
      } else {
	netmask = loadReg (curPucReg) << 16;
      }
    } 
    break;

  case NETMASK_2:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, 0xff00); // recuperer l'adresse ip
    } else {
      if (protectedState != UNPROTECTED) {
	DebugTrace ("Protected Mode");
	syslog (LOG_WARN, "netmask2: Protected Mode", servoNum);
	return FALSE;
      } else {
	netmask |= loadReg (curPucReg);
      }
    } 
    break;

  case PWMMIN:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, servoGetPwmMin (servoNum)); // recuperer l'adresse ip
    } else {
      if (protectedState != UNPROTECTED) {
	DebugTrace ("Protected Mode");
	syslog (LOG_WARN, "pwmmin: Protected Mode", servoNum);
	return FALSE;
      } else {
	servoSetPwmMin (servoNum, loadReg (curPucReg));
      }
    } 
    break;


  case PWMMAX:
    if (eMode == MB_REG_READ) {
      storeReg (curPucReg, servoGetPwmMax (servoNum)); // recuperer l'adresse ip
    } else {
      if (protectedState != UNPROTECTED) {
	syslog (LOG_WARN, "Protected Mode", servoNum);
	DebugTrace ("pwmmax: Protected Mode");
	return FALSE;
      } else {
	servoSetPwmMax (servoNum, loadReg (curPucReg));
      }
    } 
    break;

 
  case PROTECTED_REG_HOLDING_END:
    break;
  }

  return TRUE;
}

static bool_t getRegInputValue (uint16_t iRegIndex, uint8_t **curPucReg)
{
  const RegInput type = iRegIndex % REG_INPUT_END;
  const uint32_t servoNum = iRegIndex / REG_INPUT_END;
  float val;

  if (iRegIndex > (SERVO_COUNT*REG_INPUT_END)) {
    syslog (LOG_WARN, "servo>5 : %d", servoNum);
    return FALSE;
  }

  if (iRegIndex == (SERVO_COUNT*REG_INPUT_END)) {
    // last value of reg input is (nominal 5v) power supply voltage
    storeReg (curPucReg, floatToRegFact1000 (analogGet5VoltPowerVoltage()));
    return TRUE;
  }

  /* if (servoNum >= SERVO_COUNT) { */
  /*   syslog (LOG_WARN, "servo>5 : %d", servoNum); */
  /*   return FALSE; */
  /* } */

  switch (type) {
  case POS_COURANTE:
    storeReg (curPucReg, normalisedToReg (servoGetCurrentPos (servoNum)));
    break;
    
  case VITESSE_COURANTE:
    val = servoGetCurrentSpeed (servoNum);
    if (val == -1.0f) {
      storeReg (curPucReg, (1<<15)-1);
    } else {
      storeReg (curPucReg, floatToRegFact1000 (val));
    }
    break;
    
  case POSITION_PARK:
    storeReg (curPucReg, normalisedToReg (servoGetParkPos (servoNum)));
    break;
    
  case POS_POTAR_SERVO_RAW:
    storeReg (curPucReg, normalisedToReg (analogGetRawPos (servoNum)));
    break;

  case POS_POTAR_SERVO_HOMOGENE:
    storeReg (curPucReg, normalisedToReg (getReadPosNormalised (servoNum)));
    break;
    
    // in milliamp
  case INTENSITE_SERVO:
    storeReg (curPucReg, floatToRegFact1000(analogGetCurrentIntensity (servoNum)));
    break;
     
  case  REG_INPUT_END:
    break;
  }

  return TRUE;
}


static bool_t getOrSetCoilValue (uint16_t iRegIndex,  uint8_t *curPucReg, eMBRegisterMode eMode)
{
  const  RegCoil type = iRegIndex % REG_COIL_END;
  const  uint32_t servoNum = iRegIndex / REG_COIL_END;

  if (servoNum >= SERVO_COUNT) 
    return FALSE;
  
  switch (type) {
  case SETPARK:
    if (eMode == MB_REG_READ) {
      xMBUtilSetBits (curPucReg, iRegIndex, 1, 0);
      setLastValue (0);
    } else {  
      setLastValue (xMBUtilGetBits (curPucReg, iRegIndex, 1));
      if (xMBUtilGetBits (curPucReg, iRegIndex, 1)) {
	for (uint32_t idx=0; idx<SERVO_COUNT; idx++) 
	  servoSetCurrentPosAsPark(idx);
      }
    }
    break;
    
  case GOTO_PARK:
    if (eMode == MB_REG_READ) {
      xMBUtilSetBits (curPucReg, iRegIndex, 1, 0);
      setLastValue (0);
    } else {  
      setLastValue (xMBUtilGetBits (curPucReg, iRegIndex, 1));
      if (xMBUtilGetBits (curPucReg, iRegIndex, 1)) {
	for (uint32_t idx=0; idx<SERVO_COUNT; idx++) 
	  servoPark(idx);
      }
    }
    break;
    
    
  case REG_COIL_END:
    break;
  }
  
  return TRUE; 
}

static bool_t getDiscreteValue (uint16_t iRegIndex,  uint8_t *curPucReg)
{
  const RegDiscrete type = iRegIndex % REG_DISCRETE_END;
  const uint32_t servoNum = iRegIndex / REG_DISCRETE_END;

  if (servoNum >= SERVO_COUNT) {
    syslog (LOG_WARN, "servo>5 : %d", servoNum);
    return FALSE;
  }
  
  switch (type) {
  case JAMED:
    xMBUtilSetBits (curPucReg, iRegIndex, 1, servoGetState(servoNum) == ADS_Jam);
    setLastValue (servoGetState(servoNum) == ADS_Jam);
    break;

  case ENABLED:
    xMBUtilSetBits (curPucReg, iRegIndex, 1, isServoEngaged(servoNum));
    setLastValue (isServoEngaged(servoNum));
    break;

  case MOVING:
      xMBUtilSetBits (curPucReg, iRegIndex, 1, isServoMoving(servoNum));
      setLastValue (isServoMoving(servoNum));
    break;
     
  case BUTEE_MIN: 
    xMBUtilSetBits (curPucReg, iRegIndex, 1, isAtLowStopPos (servoNum));
    setLastValue (isAtLowStopPos (servoNum));
    break;
    
  case BUTEE_MAX:
    xMBUtilSetBits (curPucReg, iRegIndex, 1,  isAtHighStopPos (servoNum));
    setLastValue (isAtHighStopPos (servoNum));
    break;
    
  case REG_DISCRETE_END:
    break;
  }
  
  return TRUE; 
}

const char* getLastFunctionL1 (void)
{
  switch (modbusFunction)  {
  case FUNC_NONE:
    chsnprintf (l1, sizeof(l1), "** En attente du   **");
    chsnprintf (l2, sizeof(l2), "** Premier Message **");
    chsnprintf (l3, sizeof(l3), "---------------------");
    break;

    case FUNC_HOLDING:
      chsnprintf (l1, sizeof(l1), "%d Reg %s @ %d",
		  lastNbRegs,
		  modbusAction == MB_REG_READ ? "READ" : "WRITE",
		  lastAddress);
      
      chsnprintf (l2, sizeof(l2), "%s",
		  lastStatus == MB_ENOERR ? "OK" : 
		  lastStatus == MB_ENOREG ? "adresse incorrecte" : "valeur incorrecte");
      
      chsnprintf (l3, sizeof(l3), "last reg val: %d", lastValue);
      break;
      
    case FUNC_INPUT:
      chsnprintf (l1, sizeof(l1), "%d Input @ %d", lastNbRegs, lastAddress);
      
      chsnprintf (l2, sizeof(l2), "%s",
		  lastStatus == MB_ENOERR ? "OK" : 
		  lastStatus == MB_ENOREG ? "adresse incorrecte" : "valeur incorrecte");
      chsnprintf (l3, sizeof(l3), "last input val: %d", lastValue);
      break;
      
  case FUNC_COIL:
    chsnprintf (l1, sizeof(l1), "%d Coil %s @ %d",
		lastNbRegs,
		modbusAction == MB_REG_READ ? "READ" : "WRITE",
		lastAddress);
    
    chsnprintf (l2, sizeof(l2), "%s",
		lastStatus == MB_ENOERR ? "OK" : 
		lastStatus == MB_ENOREG ? "adresse incorrecte" : "valeur incorrecte");
    chsnprintf (l3, sizeof(l3), "last coil val: %s", lastValue ? "VRAI" : "FAUX");
    break;
  

  case FUNC_DISCRETE:
    chsnprintf (l1, sizeof(l1), "%d Discrete @ %d", lastNbRegs, lastAddress);
    
    chsnprintf (l2, sizeof(l2), "%s",
		lastStatus == MB_ENOERR ? "OK" : 
		lastStatus == MB_ENOREG ? "adresse incorrecte" : "valeur incorrecte");
    chsnprintf (l3, sizeof(l3), "last discrete val: %s", lastValue ? "VRAI" : "FAUX");
    break;
  }
  
  l1[sizeof(l1)-1] =  l2[sizeof(l2)-1] =0;
  return l1;
}

const char* getLastFunctionL2 (void)
{
  return l2;
}

const char* getLastFunctionL3 (void)
{
  return l3;
}


static void setLastFunction (ModbusFunction f)
{
   modbusFunction = f;
}

static void setLastAction (eMBRegisterMode m)
{
   modbusAction = m;
}

static void setLastAddress (uint16_t a)
{
   lastAddress = a;
}

static void setLastNbReg  (uint16_t n)
{
   lastNbRegs = n;
}

static void setLastValue  (uint16_t n)
{
   lastValue = n;
}
