#ifndef __JBUS_ETHER_H__
#define __JBUS_ETHER_H__
#include "ch.h"
#include "modbus/port/port.h"
#include "modbus/include/mb.h"
#include "modbus/include/mbport.h"
#include "eeprom.h"




bool_t initJbusEther (void);
msg_t thdJbusEtherIO(void *arg) ;

bool_t setIPAddress (const char* ipv4Str);
bool_t setNetmask   (const char* ipv4Str);
bool_t setGateway   (const char* ipv4Str);

uint32_t getIPAddress (uint32_t byte);
uint32_t getNetmask (uint32_t byte);
uint32_t getGateway (uint32_t byte);
uint32_t getMacEther (uint32_t byte);

ErrorCond     getJbusEtherStatesFromEeprom (void);
ErrorCond     storeJbusEtherStatesToEeprom (void);



#endif // __JBUS_ETHER_H__
