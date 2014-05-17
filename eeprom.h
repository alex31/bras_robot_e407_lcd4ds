#ifndef __EEPROM_H__
#define __EEPROM_H__
#include "ch.h"

/*

  TODO : tester si on depasse la taille du secteur, si oui : 
sauvegarder chaque type en ram, effacer le secteur de flash,
et reflasher les dernieres valeurs pour chaques types

 */

/* public function prototypes -----------------------------------------------*/
typedef enum  {PROG_OK=0, ERASE_ERR, PROGRAM_ERR, VERIFY_READ_ERROR, 
	       SECTOR_FULL_ERR, NOT_FOUND, SIZES_DIFFER, 
	       PROGRAM_MODIFIED, MALLOC_ERR} ErrorCond;


// new enum can be added, but never be removed. If one value is no more used, 
// prefix DISABLED before his name. And never change enum order.
// 
typedef enum  {EEPROM_SERVO=0, EEPROM_JBUS_SERIAL, EEPROM_JBUS_ETHER, 
	       DISABLED_EEPROM_IOMODE, EEPROM_PWM_CALIBRATION,
	       EEPROM_STRUCT_LAST_ITEM} EepromStructType;

typedef struct objInEeprom {
  struct objInEeprom *nextObjPtr;
  EepromStructType eepromStructType;
  uint32_t dataLen;
  uint32_t ramAdress;
  uint8_t data[0];
} objInEeprom;

int testFlash(void);

ErrorCond eepromErase (void);
ErrorCond memcopyToFlash (const void *from, const void *flashAddr, 
			 size_t length, bool_t eraseBefore);
ErrorCond eepromStore (EepromStructType eepromStructType, const void *data, 
		       uint32_t len);
ErrorCond eepromLoad (EepromStructType eepromStructType, void *data, 
		       uint32_t len);
size_t eepromGetLenOfType (EepromStructType eepromStructType);
size_t eepromStatus (void); // return free eeprom space
ErrorCond eepromWipe (void);

#endif // __EEPROM_H__
