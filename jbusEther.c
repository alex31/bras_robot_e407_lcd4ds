#include "jbusEther.h"
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
#include "lwipthread.h"
#include <ctype.h>



/* ------------------------ LWIP includes --------------------------------- */
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/memp.h"

/* ----------------------- Static Function Declaration ------------------------------------------*/
struct IPV4_Val;
static bool_t inet_aton(const char *cp, struct IPV4_Val *addr);
static void _setIPAddress (uint8_t a, uint8_t b, uint8_t c, uint8_t d);
static void _setNetmask (uint8_t a, uint8_t b, uint8_t c, uint8_t d);
static void _setGateway (uint8_t a, uint8_t b, uint8_t c, uint8_t d);


/* ----------------------- Defines ------------------------------------------*/


/* ----------------------- Static variables ---------------------------------*/
struct  IPV4_Val {
  uint8_t a;
  uint8_t b;
  uint8_t c;
  uint8_t d;
} ;


typedef struct {
  uint8_t		  macaddress[6];
  struct lwipthread_opts ipOpts;
} JbusEtherState;



static JbusEtherState 
jbusEtherStates = {
  .macaddress = {LWIP_ETHADDR_0, LWIP_ETHADDR_1, LWIP_ETHADDR_2,
		 LWIP_ETHADDR_3, LWIP_ETHADDR_4, LWIP_ETHADDR_5},
  .ipOpts = {.macaddress = jbusEtherStates.macaddress,
	     .address = POINT_TO_IP4(192,168,1,20),
	     .netmask = POINT_TO_IP4(255,255,255,0),
	     .gateway =  POINT_TO_IP4(192,168,1,1)
  }
};

static  Thread *		 lwipThd=NULL;


// === PROTECTED HOLDING ===
// test with modpoll -m tcp  -r 1666 -c 20 -t 4  192.168.2.20
// === REG HOLDING ===
// test with modpoll -m tcp  -r 100 -c 19 -t 4  192.168.2.20
// === REG INPUT ===
// test with modpoll -m tcp  -r 200 -c 25 -t 3  192.168.2.20
// === COIL ===
// modpoll -m tcp  -r 300 -c 10 -t 0  192.168.2.20
// === DISCRETE ===
// modpoll -m tcp  -r 400 -c 25 -t 1  192.168.2.20

/* ----------------------- Start implementation -----------------------------*/




bool_t initJbusEther (void)
{
  eMBErrorCode    eStatus;
  
 
  /*
   * Creates the LWIP threads (it changes priority internally).
   */

  getJbusEtherStatesFromEeprom ();
    
  // generate mac address from uniq cpu id so won't have to configure anything
  // to be sure that every card on the subnet will have differents mac address
  for (uint32_t i=0; i<3; i++) {
    jbusEtherStates.ipOpts.macaddress[i+3] = UniqProcessorId[i*2];
  }

  if (lwipThd == NULL)  {
    lwipThd = chThdCreateStatic(wa_lwip_thread, LWIP_THREAD_STACK_SIZE, NORMALPRIO+2,
    				lwip_thread, &(jbusEtherStates.ipOpts));
  }

  if( eMBTCPInit( MB_TCP_PORT_USE_DEFAULT ) != MB_ENOERR ) {
    DebugTrace ("ERROR eMBTCPInit : can't initialize modbus stack");
    return FALSE;
  }
    
    
  if ((eStatus = eMBSetSlaveID (0x42, TRUE, UniqProcessorId, 
				UniqProcessorIdLen))  != MB_ENOERR) {
    DebugTrace ("eMBSetSlaveID Failed err=%d", eStatus);
    return FALSE;
  }
    
  if (eMBEnable () != MB_ENOERR) {
    DebugTrace ("ERROR eMBEnable : can't initialize modbus stack");
    return FALSE;
  }


 
  return TRUE;
}


/*
 * Jbus ethernet I/O thread
 */


msg_t thdJbusEtherIO(void *arg)
{
  (void)arg;
  eMBErrorCode    xStatus;

  chRegSetThreadName("thd JBUS Ether IO");
  while  (initJbusEther () != TRUE) {
    chThdSleepMilliseconds(1000);
    if (chThdShouldTerminate())
      goto cleanAndExit;
  }

  do {
    xStatus = eMBPoll ();
  }  while ( (xStatus == MB_ENOERR) && (!chThdShouldTerminate()) );

    
 cleanAndExit:
  eMBDisable ();
  eMBClose ();
  
  /* if (lwipThd != NULL) { */
  /*   chThdTerminate (lwipThd); */
  /*   chThdWait (lwipThd); */
  /*   lwipThd = NULL; */
  /* } */
  DebugTrace ("thdJbusIO Ether Thread Is stopping");
    
  return 0;
}




ErrorCond     getJbusEtherStatesFromEeprom (void)
{
  ErrorCond res=  eepromLoad (EEPROM_JBUS_ETHER, &jbusEtherStates, sizeof(jbusEtherStates));
  jbusEtherStates.ipOpts.macaddress =  jbusEtherStates.macaddress;

  return res;
}


ErrorCond     storeJbusEtherStatesToEeprom (void)
{
  return eepromStore (EEPROM_JBUS_ETHER, &jbusEtherStates, sizeof(jbusEtherStates));
}

static void _setIPAddress (uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
  jbusEtherStates.ipOpts.address =  POINT_TO_IP4(a,b,c,d);
}

static void _setNetmask (uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{ 
  jbusEtherStates.ipOpts.netmask =  POINT_TO_IP4(a,b,c,d);
}

static void _setGateway (uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
  jbusEtherStates.ipOpts.gateway =  POINT_TO_IP4(a,b,c,d);
}

bool_t setIPAddress (const char* ipv4Str)
{
  struct  IPV4_Val iv;
  bool_t ok =  inet_aton(ipv4Str, &iv);
  if (ok) {
    _setIPAddress (iv.a, iv.b, iv.c, iv.d);
  }

  return ok;
}

bool_t setNetmask   (const char* ipv4Str)
{
 struct  IPV4_Val iv;
  bool_t ok =  inet_aton(ipv4Str, &iv);
  if (ok) {
   _setNetmask  (iv.a, iv.b, iv.c, iv.d);
  }

  return ok;
}

bool_t setGateway   (const char* ipv4Str)
{
 struct  IPV4_Val iv;
  bool_t ok =  inet_aton(ipv4Str, &iv);
  if (ok) {
    _setGateway (iv.a, iv.b, iv.c, iv.d);
  }

  return ok;
}

uint32_t getIPAddress (uint32_t byte)
{
  const uint32_t val = jbusEtherStates.ipOpts.address;
  return  (val & (0xff << (byte*8))) >> (byte*8);
}


uint32_t getNetmask (uint32_t byte)
{
  const uint32_t val = jbusEtherStates.ipOpts.netmask;
  return  (val & (0xff << (byte*8))) >> (byte*8);
}


uint32_t getGateway (uint32_t byte)
{ 
  const uint32_t val =jbusEtherStates.ipOpts.gateway;
  return  (val & (0xff << (byte*8))) >> (byte*8);
}


uint32_t getMacEther (uint32_t byte)
{
  if (byte > 5)
    return 0;
  else
    return jbusEtherStates.macaddress[byte];
}

static bool_t inet_aton(const char *cp, struct IPV4_Val *addr)
{
  uint32_t val;
  int base, n;
  uint32_t c;
  uint32_t parts[4];
  uint32_t *pp = parts;
  
  c = *cp;
  for (;;) {
    /*
     * Collect number up to ``.''.
     * Values are specified as for C:
     * 0x=hex, 0=octal, isdigit=decimal.
     */
    if (!isdigit(c))
     return FALSE;
    val = 0; base = 10;
    if (c == '0') {
      c = *++cp;
      if (c == 'x' || c == 'X')
	base = 16, c = *++cp;
      else
	base = 8;
    }
    for (;;) {
      if (isascii(c) && isdigit(c)) {
	val = (val * base) + (c - '0');
	c = *++cp;
      } else if (base == 16 && isascii(c) && isxdigit(c)) {
	val = (val << 4) |
	  (c + 10 - (islower(c) ? 'a' : 'A'));
	c = *++cp;
      } else
	break;
    }
    if (c == '.') {
      /*
       * Internet format:
       *	a.b.c.d
       *	a.b.c	(with c treated as 16 bits)
       *	a.b	(with b treated as 24 bits)
       */
      if (pp >= parts + 3)
	return FALSE;
      *pp++ = val;
      c = *++cp;
    } else
      break;
  }
  /*
   * Check for trailing characters.
   */
  if (c != '\0' && (!isascii(c) || !isspace(c)))
   return FALSE;
  /*
   * Concoct the address according to
   * the number of parts specified.
   */
  n = pp - parts + 1;
  switch (n) {

  case 0:
   return FALSE;		/* initial nondigit */

  case 1:				/* a -- 32 bits */
    break;

  case 2:				/* a.b -- 8.24 bits */
    if ((val > 0xffffff) || (parts[0] > 0xff))
     return FALSE;
    val |= parts[0] << 24;
    break;

  case 3:				/* a.b.c -- 8.8.16 bits */
    if ((val > 0xffff) || (parts[0] > 0xff) || (parts[1] > 0xff))
     return FALSE;
    val |= (parts[0] << 24) | (parts[1] << 16);
    break;

  case 4:				/* a.b.c.d -- 8.8.8.8 bits */
    if ((val > 0xff) || (parts[0] > 0xff) || (parts[1] > 0xff) || (parts[2] > 0xff))
     return FALSE;
    parts[3] = val;
    val |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
    break;
  }
  
  if (addr) {
    addr->a = parts[0];
    addr->b = parts[1];
    addr->c = parts[2];
    addr->d = parts[3];
  }
  
  return TRUE;
}
