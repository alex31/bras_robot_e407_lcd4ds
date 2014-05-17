#ifndef __JBUS_COMMON_H__
#define __JBUS_COMMON_H__
#include "ch.h"
#include "modbus/port/port.h"
#include "modbus/include/mb.h"
#include "modbus/include/mbport.h"
#include "eeprom.h"

/*


eMBRegisterMode

    MB_REG_READ,                Read register values and pass to protocol stack
    MB_REG_WRITE                Update register values



eMBErrorCode

    MB_ENOERR,                   no error. 
    MB_ENOREG,                   illegal register address. 
    MB_EINVAL,                   illegal argument. 
*/

typedef enum  {FUNC_INPUT, FUNC_HOLDING, FUNC_COIL, FUNC_DISCRETE, FUNC_NONE}
  ModbusFunction;


const char* getLastFunctionL1 (void);
const char* getLastFunctionL2 (void);
const char* getLastFunctionL3 (void);


eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs );

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode );

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode );

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete );



#endif // __JBUS_COMMON_H__
