#ifndef __FLASH_H__
#define __FLASH_H__
#include "ch.h"

/*

  TODO : tester si on depasse la taille du secteur, si oui : 
sauvegarder chaque type en ram, effacer le secteur de flash,
et reflasher les dernieres valeurs pour chaques types

 */

/* public function prototypes -----------------------------------------------*/
typedef enum  {PROG_OK=0, ERASE_ERR, PROGRAM_ERR, VERIFY_READ_ERROR, 
	       SECTOR_FULL_ERR, NOT_FOUND, SIZES_DIFFER, 
	       PROGRAM_MODIFIED} ErrorCond;

typedef enum  {SERVO_STOP} FlashStructType;

typedef struct objInEeprom {
  struct objInEeprom *nextObjPtr;
  FlashStructType flashStructType;
  uint32_t dataLen;
  uint32_t ramAdress;
  uint8_t data[0];
} objInEeprom;

int testFlash(void);

ErrorCond eepromErase (void);
ErrorCond memcopyToFlash (const void *from, const void *flashAddr, 
			 size_t length, bool_t eraseBefore);
ErrorCond eepromStore (FlashStructType flashStructType, const void *data, 
		       uint32_t len);
ErrorCond eepromLoad (FlashStructType flashStructType, void *data, 
		       uint32_t len);

#endif // __FLASH_H__
