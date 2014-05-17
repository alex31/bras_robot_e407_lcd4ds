#ifndef __JBUS_485_H__
#define __JBUS_485_H__
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


bool_t initJbus485 (void);
msg_t thdJbus485IO(void *arg) ;

void setBaudRate (uint32_t baud);
void setJbusSerialMode (eMBMode	mode);
void setParity (eMBParity parity);
void setNbBitsStop ( uint8_t	nbBitsStop);
void setMbAddressFor485 (uint8_t address);
void setRegOffset (int8_t offset);

uint32_t      getBaudRate (void);
eMBMode       getJbusSerialMode (void);
eMBParity     getParity (void);
uint8_t	      getNbBitsStop (void);
uint8_t	      getMbAddressFor485 (void);
int8_t	      getRegOffset (void);
ErrorCond     getJbus485StatesFromEeprom (void);
ErrorCond     storeJbus485StatesToEeprom (void);



#endif // __JBUS_485_H__
