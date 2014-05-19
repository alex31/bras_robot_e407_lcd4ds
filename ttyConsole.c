#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "ch.h"
#include "hal.h"
#include "printf.h"
#include "microrl/microrlShell.h"
#include "ttyConsole.h"
#include "stdutil.h"
#include "globalVar.h"
#include "eeprom.h"
#include "servo_pwm.h"
#include "jbus485.h"
#include "jbusEther.h"
#include "IOmode.h"
#include "rtcAccess.h"
#include "lcdDisplay.h"
#include "picaso4Display.h"
#include "analogicIO.h"
#include "i2cMaster.h"
#include "calibration.h"




/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/
typedef enum  {ALT_PWM, ALT_POS} AltMode;


#define SHELL_WA_SIZE   THD_WA_SIZE(4096)

static void pwmTune_insert_char(uint8_t c, uint32_t mode);
static void pwmPos_insert_char (uint8_t c, uint32_t mode);
static void usage (const char * const msg);
static void wrongArcCount(const uint8_t given, const uint8_t inf, const uint8_t sup, const char * const msg);
 
#define TEST_ARGC_EQ(n,s) {if (n != argc) {wrongArcCount(argc,n,n,s); return;}}
#define TEST_ARGC_ITV(i,m,s) {if ((argc < i)||(argc > m)) {wrongArcCount(argc,i,m,s); return;}}



static const char *usage_i2c = 
  "CATEGORIE: mise au point hardware\r\n"
  "fonction de debug et mise au point du harware pour bus i2c distant\r\n"
  "necessite un gpio PCF8574 sur le bus I2C\r\n"
  "envoie une ou plusieurs valeurs 8 bits sur le bus i2c qui seront affichees sur des led du banc\r\n\r\n"
  "USAGE: i2c [once ou loop] valeur_8_bits";
static void cmd_testI2c(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  TEST_ARGC_EQ(2,usage_i2c);
  const char* i2cKind = argv[0];
  const uint8_t val = atoi (argv[1]);
  
  if (getMode() != (Mode_Analog_Pos)) {
    chprintf (chp, "Mettre le rotacteur en mode analogique pour initialiser l'i2c\r\n");
    return;
  }
  
  if (strcmp (i2cKind, "once") == 0) {
    i2cSetIO_PCF8574 (&I2CD2, val);
  } else if (strcmp (i2cKind, "loop") == 0) {
    for (uint32_t i=0; i< 8*20; i++) {
      i2cSetIO_PCF8574 (&I2CD2, RotL(val,(i%8)));
      chThdSleepMilliseconds(50);
    }
  } else {
    usage (usage_i2c);
  }
}


static const char *usage_calib = 
  "CATEGORIE: réglage\r\n"
  "cherche les pwm min et max de tous les servos en monitorant le courant\r\n"
  "consommé\r\n"
  "rempli aussi une table de correspondance position servo => pwm\r\n\r\n"
  "USAGE: calib [search store load show] ";

static void cmd_calib(BaseSequentialStream *chp, int argc,
		      const char * const argv[]) {
  TEST_ARGC_EQ(1,usage_calib);
  const char* calibKind = argv[0];

  if (strcmp (calibKind, "search") == 0) {
    if ((getMode() != Mode_Tuning) && (getMode() != IO_mode_Off)) {
      chprintf (chp, "calibration impossible si mode autre que Reglage ou Off\r\n");
    } else {
      chprintf (chp, "lancement de la calibration\r\n");
      calibratePwmRange ();
      chprintf (chp, "calibration terminée (pwm info pour voir)\r\n");
    }
  } else if (strcmp (calibKind, "store") == 0) {
     chprintf (chp, "store pwmPos in eeprom\r\n");
     storePwmByPosToEeprom ();
     chprintf (chp, "store pwmPos in eeprom OK\r\n");
  }  else if (strcmp (calibKind, "load") == 0) {
     chprintf (chp, "load pwmPos from eeprom\r\n");
     getPwmByPosFromEeprom ();
     chprintf (chp, "load pwmPos from eeprom OK\r\n");
  }  else if (strcmp (calibKind, "show") == 0) {
    for (uint32_t idx=0; idx<SERVO_COUNT; idx++) {
      chprintf (chp, "servo[%d] pwmmin=%d pwmmax=%d pos=%.3f/%.3f speed=%.2f/%.2f "
		" park=%.3f || i2cI=%.2f i2cP=%.2f\r\n", 
		idx, servoGetPwmMin(idx), servoGetPwmMax(idx), 
		servoGetCurrentPos(idx), servoGetPos(idx),
		servoGetCurrentSpeed(idx), servoGetTopSpeed(idx), 
		servoGetParkPos(idx),
		analogGetCurrentIntensity(idx),
		analogGetRawPos(idx));
    }
    
  }
  
  for (uint32_t calIdx=0; calIdx<CALIBRATION_TABLE_SIZE; calIdx +=10) {
    const float pos = (float)calIdx/512;
    chprintf (chp, "CAL[%.3f] = [%d] [%d] [%d] [%d] [%d]\r\n",
	      pos,
	      getPwmByReadPos (0, pos),
	      getPwmByReadPos (1, pos),
	      getPwmByReadPos (2, pos),
	      getPwmByReadPos (3, pos),
	      getPwmByReadPos (4, pos));
  }
  
}


static const char *usage_lcd = 
  "CATEGORIE: mise au point hardware\r\n"
  "test de l'api du nouveau protocol 4dsystem\r\n\r\n"
  "USAGE: lcd [xy pb pf ci vol wav list gci disp] valeur_8_bits";

static void cmd_testLcd(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  (void) chp;
  static oledConfig olc;
  static uint16_t handle=0;

  if (argc == 0) {
    usage (usage_lcd);
    return;
  }
  const char* lcdCmd = argv[0];

  
  if (strcmp (lcdCmd, "init") == 0) {
    oledInit (&olc, &SD6, 256000, GPIOC, GPIOC_LCD_RST, PICASO); 
    oledPrintVersion (&olc);
    oledInitSdCard (&olc);

    oledSetTextBgColorTable (&olc, 0, 0,0,0);
    oledSetTextFgColorTable (&olc, 0, 100,100,100);
    // Clock color
    oledSetTextBgColorTable (&olc, 1 , 100,100,100);
    oledSetTextFgColorTable (&olc, 1 , 0,0,0);
    // Pwm color
    oledSetTextBgColorTable (&olc, 2 , 20,40,20);
    oledSetTextFgColorTable (&olc, 2 , 30,100,30);
    // input mode color
    oledSetTextBgColorTable (&olc, 3 , 40,20,20);
    oledSetTextFgColorTable (&olc, 3 , 100,50,50);
    
    // Log INFO Color
    oledSetTextBgColorTable (&olc, 4 , 0,0,0);
    oledSetTextFgColorTable (&olc, 4 , 0,50,100);
    // Log WARNING Color
    oledSetTextBgColorTable (&olc, 5 , 0,0,0);
    oledSetTextFgColorTable (&olc, 5 , 100,100,0);
    // Log ERROR Color
    oledSetTextBgColorTable (&olc, 6 , 0,0,0);
    oledSetTextFgColorTable (&olc, 6 , 100,20,20);
    
    oledClearScreen (&olc);
    
  } else  if (strcmp (lcdCmd, "xy") == 0) {
    if (argc == 3) {
      oledGotoXY (&olc, atoi (argv[1]),  atoi (argv[2]));
    }
  }  else  if (strcmp (lcdCmd, "pb") == 0) {
    if (argc == 2) {
      oledPrintBuffer (&olc, argv[1]);
    }
  }  else  if (strcmp (lcdCmd, "pf") == 0) {
    if (argc == 2) {
      oledPrintFmt (&olc, "%s@%d:%d", argv[1], olc.curXpos, olc.curYpos);
    }
  }  else  if (strcmp (lcdCmd, "ci") == 0) {
    if (argc == 2) {
      oledUseColorIndex (&olc, atoi (argv[1]));
    }
  }  else  if (strcmp (lcdCmd, "vol") == 0) {
    if (argc == 2) {
      oledSetSoundVolume (&olc, atoi (argv[1]));
    }
  }  else  if (strcmp (lcdCmd, "wav") == 0) {
    oledPlayWav (&olc, "C.WAV");
  }  else  if (strcmp (lcdCmd, "list") == 0) {
    oledListSdCardDirectory (&olc);
  }  else  if (strcmp (lcdCmd, "gci") == 0) {
    oledOpenFile (&olc, "C2.GCI", &handle);
    DebugTrace ("gci handle = 0x%x", handle);
  }  else  if (strcmp (lcdCmd, "disp") == 0) {
    if (argc == 2) {
      oledDisplayGci (&olc, handle, atoi (argv[1]));
    }
  } else {
    usage (usage_lcd);
  }
}

static const char *usage_adc = 
  "CATEGORIE: mise au point\r\n"
  "lecture des valeurs des adc, builtin du stm32, et distants i2c sur la carte robot\r\n\r\n"
  "USAGE: adc [blt ou i2c ou all]";
static void cmd_testAdc(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  TEST_ARGC_EQ(1,usage_adc);

  const char* adcKind = argv[0];
  if (strcmp (adcKind, "blt") == 0) {
    for (uint8_t i=0; i< SERVO_COUNT; i++) {
      chprintf (chp, "[%d] cmd=%.2f, current=%.2f\r\n", i, 
		analogGetCmd(i),  analogGetCurrentIntensity(i));
    }
  } else if (strcmp (adcKind, "i2c") == 0) {
    for (uint8_t i=0; i< SERVO_COUNT+1; i++) {
      chprintf (chp, "[%d] pos=%.3f V=%.2f\r\n", i, analogGetRawPos(i), analogGetRawPos(i)*5.01f);
    }
  } else if (strcmp (adcKind, "all") == 0) {
    for (uint8_t i=0; i< SERVO_COUNT; i++) {
      chprintf (chp, "[%d] cmd=%.2f, current=%.2f\r\n"
		"pos=%.2f\r\n",
		i,
		analogGetCmd(i),  
		analogGetCurrentIntensity(i),
		analogGetRawPos(i));
    }
    chprintf (chp, "[5] pos=%.2f\r\n", analogGetRawPos(5));
  }  else if (strcmp (adcKind, "cal") == 0) {
    analogCalibrateCurrentIntensity ();
  } else {
    usage (usage_adc);
  }
}

static const char *usage_gpio = 
  "CATEGORIE: mise au point HARDWARE\r\n"
  "permet de passer des pins du stm32 en gpio, input ou output pour tester\r\n"
  "avec un voltmetre la presence ou absence d'une tension\r\n"
  "on peut aussi recuperer la valeur logique en entree sur les pins\r\n"
  "toutes les pins ne sont pas prevues, liste de celles qui sont manipulables:\r\n"
  "TX6, RX6, SCK, RST, BL, CS, SDA, SCL, TX, RX, L1, L2, D0 a D9\r\n\r\n"
  "USAGE: gpio mode 0 1 pour passer l'ensemble des pin en entree ou sortie\r\n"
  "       gpio [all tx6 rx6 sck rst bl cs sda scl tx rx] l1 l2 d0..d9 [0 1] pour \r\n"
  "            faire varier la valeur de sortie d'une pin \r\n";

static void cmd_testGpio(BaseSequentialStream *chp, int argc,const char * const argv[]) {
    TEST_ARGC_EQ(2,usage_gpio);

  const char* pinName = argv[0];
  const uint8_t enable = atoi(argv[1]);

  if (strcmp (pinName, "mode") == 0) {
    if (enable) {
      palSetPadMode (GPIOC, GPIOC_LCD_TX6, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOC, GPIOC_LCD_RX6, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOG, GPIOG_LCD_SCK, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOC, GPIOC_LCD_RST, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOC, GPIOC_LCD_BL, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOG, GPIOG_LCD_CS, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOF, GPIOF_I2C2_SDA, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOF, GPIOF_I2C2_SCL, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOB, GPIOB_USART3_TX, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOD, GPIOD_USART3_RX, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOD, GPIOD_USART3_RX, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOD, GPIOD_USART3_RX, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOF, GPIOF_STOP_TURRET_RIGHT, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOG, GPIOG_STOP_TURRET_LEFT, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOG, GPIOG_STOP_SHOULDER_BACK, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOG, GPIOG_STOP_SHOULDER_FRONT, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOG, GPIOG_STOP_ELBOW_HIGH, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOG, GPIOG_STOP_ELBOW_LOW, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOG, GPIOG_STOP_WRIST_HIGH, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOG, GPIOG_STOP_WRIST_LOW, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOG, GPIOG_STOP_CLAMP_OPEN, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode (GPIOG, GPIOG_STOP_CLAMP_CLOSED, PAL_MODE_OUTPUT_PUSHPULL);
    } else {
      palSetPadMode  (GPIOC, GPIOC_LCD_TX6, PAL_MODE_INPUT);
      palSetPadMode  (GPIOC, GPIOC_LCD_RX6, PAL_MODE_INPUT);
      palSetPadMode  (GPIOG, GPIOG_LCD_SCK,  PAL_MODE_INPUT);
      palSetPadMode  (GPIOC, GPIOC_LCD_RST,  PAL_MODE_INPUT);
      palSetPadMode  (GPIOC, GPIOC_LCD_BL,   PAL_MODE_INPUT);
      palSetPadMode  (GPIOG, GPIOG_LCD_CS,   PAL_MODE_INPUT);
      palSetPadMode  (GPIOF, GPIOF_I2C2_SDA, PAL_MODE_INPUT);
      palSetPadMode  (GPIOF, GPIOF_I2C2_SCL, PAL_MODE_INPUT);
      palSetPadMode  (GPIOB, GPIOB_USART3_TX, PAL_MODE_INPUT);
      palSetPadMode  (GPIOD, GPIOD_USART3_RX, PAL_MODE_INPUT);
      palSetPadMode  (GPIOF, GPIOF_STOP_TURRET_RIGHT, PAL_MODE_INPUT);
      palSetPadMode  (GPIOG, GPIOG_STOP_TURRET_LEFT, PAL_MODE_INPUT);
      palSetPadMode  (GPIOG, GPIOG_STOP_SHOULDER_BACK, PAL_MODE_INPUT);
      palSetPadMode  (GPIOG, GPIOG_STOP_SHOULDER_FRONT, PAL_MODE_INPUT);
      palSetPadMode  (GPIOG, GPIOG_STOP_ELBOW_HIGH, PAL_MODE_INPUT);
      palSetPadMode  (GPIOG, GPIOG_STOP_ELBOW_LOW, PAL_MODE_INPUT);
      palSetPadMode  (GPIOG, GPIOG_STOP_WRIST_HIGH, PAL_MODE_INPUT);
      palSetPadMode  (GPIOG, GPIOG_STOP_WRIST_LOW, PAL_MODE_INPUT);
      palSetPadMode  (GPIOG, GPIOG_STOP_CLAMP_OPEN, PAL_MODE_INPUT);
      palSetPadMode  (GPIOG, GPIOG_STOP_CLAMP_CLOSED, PAL_MODE_INPUT);

      chprintf (chp, "GPIOC, GPIOC_LCD_TX6  =%d\r\n", palReadPad (GPIOC, GPIOC_LCD_TX6));
      chprintf (chp, "GPIOC, GPIOC_LCD_RX6  =%d\r\n", palReadPad (GPIOC, GPIOC_LCD_RX6));
      chprintf (chp, "GPIOG, GPIOG_LCD_SCK,  =%d\r\n", palReadPad (GPIOG, GPIOG_LCD_SCK));
      chprintf (chp, "GPIOC, GPIOC_LCD_RST   =%d\r\n", palReadPad (GPIOC, GPIOC_LCD_RST));
      chprintf (chp, "GPIOC, GPIOC_LCD_BL    =%d\r\n", palReadPad (GPIOC, GPIOC_LCD_BL));
      chprintf (chp, "GPIOG, GPIOG_LCD_CS    =%d\r\n", palReadPad (GPIOG, GPIOG_LCD_CS));
      chprintf (chp, "GPIOF, GPIOF_I2C2_SDA  =%d\r\n", palReadPad (GPIOF, GPIOF_I2C2_SDA));
      chprintf (chp, "GPIOF, GPIOF_I2C2_SCL  =%d\r\n", palReadPad (GPIOF, GPIOF_I2C2_SCL));
      chprintf (chp, "GPIOB, GPIOB_USART3_T  =%d\r\n", palReadPad (GPIOB, GPIOB_USART3_TX));
      chprintf (chp, "GPIOD, GPIOD_USART3_R  =%d\r\n", palReadPad (GPIOD, GPIOD_USART3_RX));
    }
  } else if (strcmp (pinName, "all") == 0) {
    if (enable) {
      palSetPad (GPIOC, GPIOC_LCD_TX6);
      palSetPad (GPIOC, GPIOC_LCD_RX6);
      palSetPad (GPIOG, GPIOG_LCD_SCK);
      palSetPad (GPIOC, GPIOC_LCD_RST);
      palSetPad (GPIOC, GPIOC_LCD_BL);
      palSetPad (GPIOG, GPIOG_LCD_CS);
      palSetPad (GPIOB, GPIOB_USART3_TX);
      palSetPad (GPIOD, GPIOD_USART3_RX);
      palSetPad (GPIOF, GPIOF_STOP_TURRET_RIGHT);
      for (uint8_t i=GPIOG_STOP_CLAMP_OPEN; i<=  GPIOG_STOP_SHOULDER_BACK; i++)
	palSetPad (GPIOG, i);
    } else {
      palClearPad (GPIOC, GPIOC_LCD_TX6);
      palClearPad (GPIOC, GPIOC_LCD_RX6);
      palClearPad (GPIOG, GPIOG_LCD_SCK);
      palClearPad (GPIOC, GPIOC_LCD_RST);
      palClearPad (GPIOC, GPIOC_LCD_BL);
      palClearPad (GPIOG, GPIOG_LCD_CS);
      palClearPad (GPIOB, GPIOB_USART3_TX);
      palClearPad (GPIOD, GPIOD_USART3_RX);
      for (uint8_t i=GPIOG_STOP_CLAMP_OPEN; i<=  GPIOG_STOP_SHOULDER_BACK; i++)
	palClearPad (GPIOG, i);
    }
  } else if (strcmp (pinName, "tx6") == 0) {
    if (enable)
      palSetPad (GPIOC, GPIOC_LCD_TX6);
    else
      palClearPad (GPIOC, GPIOC_LCD_TX6);
  } else if (strcmp (pinName, "rx6") == 0) {
    if (enable)
      palSetPad (GPIOC, GPIOC_LCD_RX6);
    else
      palClearPad (GPIOC, GPIOC_LCD_RX6);
  } else if (strcmp (pinName, "sck") == 0) {
    if (enable)
      palSetPad (GPIOG, GPIOG_LCD_SCK);
    else
      palClearPad (GPIOG, GPIOG_LCD_SCK);
  } else if (strcmp (pinName, "rst") == 0) {
    if (enable)
      palSetPad (GPIOC, GPIOC_LCD_RST);
    else
      palClearPad (GPIOC, GPIOC_LCD_RST);
  } else if (strcmp (pinName, "bl") == 0) {
    if (enable)
      palSetPad (GPIOC, GPIOC_LCD_BL);
    else
      palClearPad (GPIOC, GPIOC_LCD_BL);
  } else if (strcmp (pinName, "cs") == 0) {
    if (enable)
      palSetPad (GPIOG, GPIOG_LCD_CS);
    else
      palClearPad (GPIOG, GPIOG_LCD_CS);
  } else if (strcmp (pinName, "sda") == 0) {
    if (enable)
      palSetPad (GPIOF, GPIOF_I2C2_SDA);
    else
      palClearPad (GPIOF, GPIOF_I2C2_SDA);
  } else if (strcmp (pinName, "scl") == 0) {
    if (enable)
      palSetPad (GPIOF, GPIOF_I2C2_SCL);
    else
      palClearPad (GPIOF, GPIOF_I2C2_SCL);
  } else if (strcmp (pinName, "tx") == 0) {
    if (enable)
      palSetPad (GPIOB, GPIOB_USART3_TX);
    else
      palClearPad (GPIOB, GPIOB_USART3_TX);
  } else if (strcmp (pinName, "rx") == 0) {
    if (enable)
      palSetPad (GPIOD, GPIOD_USART3_RX);
    else
      palClearPad (GPIOD, GPIOD_USART3_RX);
  } else if (strcmp (pinName, "l1") == 0) {
    if (enable) {
      palSetPad (GPIOG, GPIOG_LED_GREEN);
    }
    else {
      palClearPad (GPIOG, GPIOG_LED_GREEN);
    }
  } else if (strcmp (pinName, "l2") == 0) {
    if (enable)
      palSetPad (GPIOF, GPIOF_LED_RED);
    else
      palClearPad (GPIOF, GPIOF_LED_RED);
  } else if (strcmp (pinName, "d0") == 0) {
    if (enable) {
      palSetPad (GPIOF, GPIOF_STOP_TURRET_RIGHT);
    }
    else {
      palClearPad (GPIOF,GPIOF_STOP_TURRET_RIGHT);
    }
  } else if (strcmp (pinName, "d1") == 0) {
    if (enable)
      palSetPad (GPIOG, GPIOG_STOP_TURRET_LEFT);
    else
      palClearPad (GPIOG, GPIOG_STOP_TURRET_LEFT);
  } else if (strcmp (pinName, "d2") == 0) {
    if (enable)
      palSetPad (GPIOG, GPIOG_STOP_SHOULDER_BACK);
    else
      palClearPad (GPIOG, GPIOG_STOP_SHOULDER_BACK);
  } else if (strcmp (pinName, "d3") == 0) {
    if (enable)
      palSetPad (GPIOG, GPIOG_STOP_SHOULDER_FRONT);
    else
      palClearPad (GPIOG, GPIOG_STOP_SHOULDER_FRONT);
  } else if (strcmp (pinName, "d4") == 0) {
    if (enable)
      palSetPad (GPIOG, GPIOG_STOP_ELBOW_HIGH);
    else
      palClearPad (GPIOG, GPIOG_STOP_ELBOW_HIGH);
  } else if (strcmp (pinName, "d5") == 0) {
    if (enable)
      palSetPad (GPIOG, GPIOG_STOP_ELBOW_LOW);
    else
      palClearPad (GPIOG, GPIOG_STOP_ELBOW_LOW);
  } else if (strcmp (pinName, "d6") == 0) {
    if (enable)
      palSetPad (GPIOG, GPIOG_STOP_WRIST_HIGH);
    else
      palClearPad (GPIOG, GPIOG_STOP_WRIST_HIGH);
  } else if (strcmp (pinName, "d7") == 0) {
    if (enable)
      palSetPad (GPIOG, GPIOG_STOP_WRIST_LOW);
    else
      palClearPad (GPIOG, GPIOG_STOP_WRIST_LOW);
  } else if (strcmp (pinName, "d8") == 0) {
    if (enable)
      palSetPad (GPIOG, GPIOG_STOP_CLAMP_OPEN);
    else
      palClearPad (GPIOG, GPIOG_STOP_CLAMP_OPEN);
  } else if (strcmp (pinName, "d9") == 0) {
    if (enable)
      palSetPad (GPIOG, GPIOG_STOP_CLAMP_CLOSED);
    else
      palClearPad (GPIOG, GPIOG_STOP_CLAMP_CLOSED);
  } else {
    usage (usage_gpio);
  }
}

static const char *usage_pwm =
  "CATEGORIE: mise au point et reglages initiaux\r\n"
  "envoi des parametres au module de commande, ou passage en shell interactif\r\n\r\n"
  "Usage: pwm N varname value \r\nor pwm tune \r\nor pwm info\r\nor pwm pos\r\n"
  "avec N compris entre 0 et 4\r\n"
  "et varname = [pwmmin, pwmmax, pos, speed, enable, disable, invmm(inversion pwm min et max)]\r\n"
  "tune : passage en shell interractif pour reglage des pwm min et max avec les fleches du clavier\r\n"
  "pos :  passage en shell interractif pour commande des servos avec les fleches du clavier\r\n"
  "info : affiche les valeurs instantanees du module\r\n";
static void cmd_pwm(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  const uint32_t pwmIdx = atoi (argv[0]);
  const char* varName = argv[1];
  const char* varVal = argv[2];
  static const char pwmStateChar[] = {'I', 'J', 'A', 'T', 'D'};

  if (pwmIdx >= SERVO_COUNT) {
    usage (usage_pwm);
    return;
  }

 if (argc == 1) {
   if (strcmp (argv[0], "info") == 0) {
     chprintf (chp, "5V power supply : %.2f\r\n",  analogGet5VoltPowerVoltage ());
     for (uint32_t idx=0; idx<SERVO_COUNT; idx++) {
       chprintf (chp, "servo[%d] pwmmin=%d pwmmax=%d pos=%.3f/%.3f speed=%.2f/%.2f state=%c"
		 " park=%.3f || i2cI=%.2f i2cP=%.2f\r\n", 
		 idx, servoGetPwmMin(idx), servoGetPwmMax(idx), 
		 servoGetCurrentPos(idx), servoGetPos(idx),
		 servoGetCurrentSpeed(idx), servoGetTopSpeed(idx), pwmStateChar[servoGetState(idx)],
		 servoGetParkPos(idx),
		 analogGetCurrentIntensity(idx),
		 analogGetRawPos(idx));
     }
   } else  if (strcmp (argv[0], "tune") == 0) {
     modeAlternate (&pwmTune_insert_char, ALT_PWM);
     chprintf (chp, "mode reglage des pwm min et max\r\n");
     chprintf (chp, "\r\n");
     chprintf (chp, "touche Orig		: reglage du PWM MIN\r\n");
     chprintf (chp, "touche Fin		: reglage du PWM MAX\r\n");
     chprintf (chp, "touche PgSuiv		: passage au servo suivant\r\n");
     chprintf (chp, "touche PgPrev 		: passage au servo precedent\r\n");
     chprintf (chp, "fleche vers le haut 	: vers la butee\r\n");
     chprintf (chp, "fleche vers le bas  	: eloigne de la butee\r\n");
     chprintf (chp, "fleche vers la droite 	: vers la butee en reglage fin (+lent)\r\n");
     chprintf (chp, "fleche vers la gauche 	: vers eloigne de la butee en reglage fin (+lent)\r\n");
     chprintf (chp, "deux fois ESCAPE 	: retour au SHELL\r\n");
     chprintf (chp, "\r\n");
     chprintf (chp, "a vous de jouer\r\n\r\n");
     
   } else  if (strcmp (argv[0], "pos") == 0) {
     modeAlternate (&pwmPos_insert_char, ALT_POS);
     chprintf (chp, "mode commande des servos par le clavier\r\n");
     chprintf (chp, "\r\n");
     chprintf (chp, "touche PgSuiv		: passage au servo suivant\r\n");
     chprintf (chp, "touche PgPrev 		: passage au servo precedent\r\n");
     chprintf (chp, "fleche vers le haut 	: mouvement rapide de pos 0 vers 1\r\n");
     chprintf (chp, "fleche vers le bas  	: mouvement rapide de pos 1 vers 0\r\n");
     chprintf (chp, "fleche vers la droite 	: mouvement lent de pos 0 vers 1\r\n");
     chprintf (chp, "fleche vers la gauche 	: mouvement lent de pos 1 vers 0\r\n");
     chprintf (chp, "deux fois ESCAPE 	: retour au SHELL\r\n");
     chprintf (chp, "\r\n");
     chprintf (chp, "a vous de jouer\r\n\r\n");
   }
   
   return;
 }
 
 
 if (argc != 3) {
   usage (usage_pwm);
   return;
 }
 
  if (strcmp (varName, "pos") == 0) {
    const float varVal_f = atof_m (varVal);
    servoSetPos (pwmIdx, varVal_f);
  } else if (strcmp (varName, "speed") == 0) {
    const float varVal_f = atof_m (varVal);
    servoSetTopSpeed (pwmIdx, varVal_f);
  } else if (strcmp (varName, "pwmmin") == 0) {
    const uint32_t varVal_i = atoi (varVal);
    servoSetPwmMin (pwmIdx, varVal_i);
  }  else if (strcmp (varName, "pwmmax") == 0) {
    const uint32_t varVal_i = atoi (varVal);
    servoSetPwmMax (pwmIdx, varVal_i);
  } else if (strcmp (varName, "min") == 0) {
    const uint32_t varVal_i = atoi (varVal);
    servoSetPwmMin (pwmIdx, varVal_i);
  }  else if (strcmp (varName, "max") == 0) {
    const uint32_t varVal_i = atoi (varVal);
    servoSetPwmMax (pwmIdx, varVal_i);
  }  else if (strcmp (varName, "enable") == 0) {
    servoEngage (pwmIdx);
  }  else if (strcmp (varName, "disable") == 0) {
    servoDisengage (pwmIdx);
  }  else if (strcmp (varName, "invmm") == 0) {
    inversePwmMinAndMax (pwmIdx);
  } else {
    usage (usage_pwm);
  }
}

static const char *usage_rtc =   
  "CATEGORIE: reglages initiaux\r\n"
  "reglage de l'horloge interne sauvegardee par pile\r\n"
  "\r\n"
  "Usage: rtc [Hour Minute Second Year monTh Day day_of_Week Adjust] value or\r\n"
  "Usage: rtc  Hour Minute Second Year monTh Day\r\n";


static void cmd_rtc(BaseSequentialStream *chp, int argc,const char * const argv[])
{
  if ((argc != 0) && (argc != 2) && (argc != 6)) {
    usage (usage_rtc);
    return;
  }
 
  if (argc == 2) {
    const char timeVar = (char) tolower ((int) *(argv[0]));
    const int32_t varVal = atoi (argv[1]);
    
    switch (timeVar) {
    case 'h':
      setHour (varVal);
      break;
      
    case 'm':
       setMinute (varVal);
      break;
      
    case 's':
      setSecond (varVal);
      break;
      
    case 'y':
       setYear (varVal);
      break;
      
    case 't':
       setMonth (varVal);
      break;
      
    case 'd':
       setMonthDay (varVal);
      break;

    case 'w':
       setWeekDay (varVal);
      break;

    case 'a':
      {
	int32_t newSec =getSecond() + varVal;
	if (newSec > 59) {
	  int32_t newMin =getMinute() + (newSec/60);
	  if (newMin > 59) {
	    setHour ((getHour()+(newMin/60)) % 24);
	    newMin %= 60;
	  }
	  setMinute (newMin);
	}
	if (newSec < 0) {
	  int32_t newMin =getMinute() + (newSec/60)-1;
	  if (newMin < 0) {
	    setHour ((getHour()-(newMin/60)-1) % 24);
	    newMin %= 60;
	  }
	  setMinute (newMin);
	}
	setSecond (newSec % 60);
      }
      break;
      
    default:
      usage (usage_rtc);
     }
  } else if (argc == 6) {
    setHour (atoi(argv[0]));
    setMinute (atoi(argv[1]));
    setSecond (atoi(argv[2]));
    setYear (atoi(argv[3]));
    setMonth (atoi(argv[4]));
    setMonthDay (atoi(argv[5]));
  }

  chprintf (chp, "RTC : %s %.02u/%.02u/%.04u  %.02u:%.02u:%.02u\r\n",
	    getWeekDayAscii(), getMonthDay(), getMonth(), getYear(),
	    getHour(), getMinute(), getSecond());
}

static const char *usage_jbus =
  "CATEGORIE: reglages initiaux\r\n"
  "reglages des parametres de communication jbus\r\n"
  "en mode liaison serie rs485 et en mode tcp/ip au dessus d'ethernet\r\n"
  "Affichage :\r\n"
  "° info\r\n"
  "PARAMETRES JBUS :\r\n"
  "° adresse jbus\r\n"
  "° decalage index registre\r\n"
  "PARAMETRES liaison SERIE :\r\n"
  "° vitesse liaison serie\r\n"
  "° parite\r\n"
  "° nombre de bits de stop\r\n"
  "° protocole ascii ou rtu\r\n"
  "PARAMETRES ethernet et TCP/IP :\r\n"
  "° adresse ip\r\n"
  "° netmask\r\n"
  "° routeur par defaut\r\n"
  "SAUVEGARDE et RECUPERATION des parametres en eeprom\r\n"
  "° save\r\n"
  "° load\r\n\r\n"
  "USAGE: jbus command [opt value]\r\n"
  "with command = info|baud|nopar|pair|impair|stopbit[1,2]|proto[ascii,rtu]|"
  "|\r\naddr[slave addr]|save|load|ip [a.b.c.d]| netmask [a.b.c.d]|offset|"
  "gateway [a.b.c.d]\r\n";

static void cmd_jbus(BaseSequentialStream *chp, int argc,const char * const argv[]) 
{
  
  if ((argc == 0) || (argc > 5)) {
    usage (usage_jbus);
    return;
  }
  
  const char* varName = argv[0];
  const char* varVal = "none";
  if (argc == 2)
    varVal = argv[1];
  

  if (strcmp (varName, "baud") == 0) {
    const uint32_t varVal_i = atoi (varVal);
    setBaudRate (varVal_i);
  } else if (strcmp (varName, "nopar") == 0) {
    setParity (MB_PAR_NONE);
  }  else if (strcmp (varName, "pair") == 0) {
    setParity (MB_PAR_EVEN);
  }  else if (strcmp (varName, "impair") == 0) {
    setParity (MB_PAR_ODD);
  } else if (strcmp (varName, "stopbit") == 0) {
    const uint8_t varVal_i = atoi (varVal);
    setNbBitsStop (varVal_i);
  } else if (strcmp (varName, "proto") == 0) {
    if (strcmp (varVal, "ascii") == 0) {
      setJbusSerialMode (MB_ASCII);
    } else if (strcmp (varVal, "rtu") == 0) {
      setJbusSerialMode (MB_RTU);
    } else {
      chprintf (chp, "jbus proto incorrect, doit etre : ascii, rtu\r\n");
    }

  } else if (strcmp (varName, "addr") == 0) {
    const uint8_t varVal_i = atoi (varVal);
    setMbAddressFor485 (varVal_i);
  } else if (strcmp (varName, "offset") == 0) {
    const int8_t varVal_i = atoi (varVal);
    setRegOffset (varVal_i);
  } else if (strcmp (varName, "ip") == 0) {
    if  ((argc != 2) || (setIPAddress (argv[1]) != TRUE)) {
      chprintf (chp, "jbus ip incorrecte, doit etre de la forme "
		"jbus ip aaa.bbb.ccc.ddd\r\n");
    }
  } else if (strcmp (varName, "netmask") == 0) {
    if  ((argc != 2) || (setNetmask (argv[1]) != TRUE)) {
      chprintf (chp, "jbus netmask incorrect, doit etre de la forme "
		"jbus netmask aaa.bbb.ccc.ddd\r\n");
    }
  } else if (strcmp (varName, "gateway") == 0) {
    if  ((argc != 2) || (setGateway (argv[1]) != TRUE)) {
      chprintf (chp, "jbus gateway incorrect, doit etre de la forme "
		"jbus ip aaa.bbb.ccc.ddd\r\n");
    } 
  } else if (strcmp (varName, "save") == 0) {
    storeJbus485StatesToEeprom();
    storeJbusEtherStatesToEeprom();
  }  else if (strcmp (varName, "load") == 0) {
    getJbus485StatesFromEeprom ();
    getJbusEtherStatesFromEeprom ();
    chprintf (chp, "baud[%d] parity[%d] stopbit[%d] proto[%s] addr[%d]\r\n"
	      "ip %d.%d.%d.%d netmask %d.%d.%d.%d gateway %d.%d.%d.%d\r\n"
	      "ether %x:%x:%x:%x:%x:%x\r\n",
	      getBaudRate(), getParity(), getNbBitsStop(), 
	      getJbusSerialMode() == MB_RTU ? "rtu" :  
	      getJbusSerialMode() == MB_ASCII ? "ascii" : "ether",
	      getMbAddressFor485 (),
	      getIPAddress(0),getIPAddress(1), getIPAddress(2),getIPAddress(3),
	      getNetmask(0),getNetmask(1), getNetmask(2),getNetmask(3),
	      getGateway(0), getGateway(1), getGateway(2), getGateway(3),
	      getMacEther(0),getMacEther(1), getMacEther(2),getMacEther(3),
	      getMacEther(4),getMacEther(5)
	      );
    
  }   else if (strcmp (varName, "info") == 0) {
    chprintf (chp, "baud[%d] parity[%d] stopbit[%d] proto[%s] addr[%d]\r\n"
	      "ip %d.%d.%d.%d netmask %d.%d.%d.%d gateway %d.%d.%d.%d\r\n"
	      "ether %x:%x:%x:%x:%x:%x offset %d\r\n",
	      getBaudRate(), getParity(), getNbBitsStop(), 
	      getJbusSerialMode() == MB_RTU ? "rtu" :  
	      getJbusSerialMode() == MB_ASCII ? "ascii" : "ether",
	      getMbAddressFor485 (),
	      getIPAddress(0),getIPAddress(1), getIPAddress(2),getIPAddress(3),
	      getNetmask(0),getNetmask(1), getNetmask(2),getNetmask(3),
	      getGateway(0), getGateway(1), getGateway(2), getGateway(3),
	      getMacEther(0),getMacEther(1), getMacEther(2),getMacEther(3),
	      getMacEther(4),getMacEther(5), getRegOffset()
	      );
  }  else {
    usage (usage_jbus);
  }
}

static const char *usage_mem =
  "CATEGORIE: surveillance\r\n"
  "info sur l'etat de la memoire ram libre\r\n\r\n"
  "USAGE : mem\r\n";
static void cmd_mem(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  size_t n, size;
  (void)argv;
  (void)argc;

  usage (usage_mem);
  
  n = chHeapStatus(&ccmHeap, &size);
  chprintf (chp, "ccm buffer address : 0x%x\r\n", &ccmHeapBuffer);
  chprintf (chp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf (chp, "heap fragments   : %u\r\n", n);
  chprintf (chp, "heap free total  : %u bytes\r\n", size);
}



static const char *usage_save =
  "CATEGORIE: reglages initiaux\r\n"
  "sauve les reglages de servo et jbus dans l'eeprom\r\n\r\n"
  "USAGE : save\r\n";
static void cmd_saveEeprom(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  (void)argv;
  (void)argc;

  usage (usage_save);
  ErrorCond res = storeServoStatesToEeprom ();    
  chprintf (chp, "sauve conf servo dans eeprom res=%s free=%d\r\n", 
	    res ? "ECHOUE" : "REUSSI", eepromStatus());
  res = storeJbus485StatesToEeprom();
  chprintf (chp, "sauve conf jbus rs485 dans eeprom res=%s free=%d\r\n", 
	    res ? "ECHOUE" : "REUSSI", eepromStatus());
  res = storeJbusEtherStatesToEeprom();
  chprintf (chp, "sauve conf jbus ip dans eeprom res=%s free=%d\r\n", 
	    res ? "ECHOUE" : "REUSSI", eepromStatus());
}

static const char *usage_load =
  "CATEGORIE: reglages initiaux et mise au point\r\n"
  "recupere les reglages de servo et jbus dans l'eeprom\r\n\r\n"
  "USAGE : load\r\n";
static void cmd_loadEeprom(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  (void)argv;
  (void)argc;

  usage (usage_load);
  ErrorCond res = getServoStatesFromEeprom ();
  chprintf (chp, "charge conf servo depuis eeprom res=%s free=%d\r\n", 
	    res ? "ECHOUE" : "REUSSI", eepromStatus());
  res = getJbus485StatesFromEeprom();
  chprintf (chp, "charge conf jbus rs485 depuis eeprom res=%s free=%d\r\n", 
	    res ? "ECHOUE" : "REUSSI", eepromStatus());
  res = getJbusEtherStatesFromEeprom();
  chprintf (chp, "charge conf jbus ip depuis eeprom res=%s free=%d\r\n", 
	    res ? "ECHOUE" : "REUSSI", eepromStatus());
}

static const char *usage_erase =
  "CATEGORIE: mise au point et maintenance\r\n"
  "reformate la memoire flash, toutes les configurations sauvees seront perdues\r\n"
  "\r\n\r\n"
  "USAGE : erase [yes]\r\n";
static void cmd_testFlashErase(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  (void)argv;
  TEST_ARGC_EQ(1,usage_erase);

  chprintf (chp, "erase flash begin\r\n");
  chThdSleepMilliseconds(500);
  int res = eepromErase();
  chprintf (chp, "erase result [-x:erase error, 0:ok]  : %d  free=%d\r\n", 
	    res, eepromStatus());
}

static const char *usage_wipe =
  "CATEGORIE: maintenance\r\n"
  "compacte la memoire flash, les configurations sauvees sont conservees\r\n"
  "\r\n\r\n"
  "USAGE : wipe [yes]\r\n";
static void cmd_wipeEeprom(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  (void)argv;
  TEST_ARGC_EQ(1,usage_wipe);

  ErrorCond res = eepromWipe ();
  chprintf (chp, "wipe eeprom res=%s free=%d\r\n", 	    
	    res ? "ECHOUE" : "REUSSI", eepromStatus());
}

static const char *usage_park =
  "CATEGORIE: mise au point\r\n"
  "met le bras robot en position de repos\r\n"
  "ou sauve la position courante comme position de repos\r\n"
  "\r\n"
  "USAGE : park : met le bras robot en position de repos\r\n"
  "        park set : sauve la position courante comme position de repos\r\n"
  "        attention : la position sauvee n'est pas mise en eeprom, il faut\r\n"
  "        faire un eeprom save pour que cette position soit sauvee\r\n";
static void cmd_park(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  (void)argv;
  (void)chp;
  TEST_ARGC_ITV(0,1, usage_park);
  
  for (uint32_t idx=0; idx<SERVO_COUNT; idx++) { 
    if (argc == 0)
      servoPark(idx);
    else
      servoSetCurrentPosAsPark(idx);
  }
}


static const char *usage_inputMode =
  "CATEGORIE: mise au point\r\n"
  "selectionne le type d'entree entre\r\n"
  "Arret, Tout ou rien, tensions analogiques, jbus RTU sur rs485\r\n"
  "jbus IP au dessus d'ethernet\r\n\r\n"
  "USAGE : mode O(ff) :     Arret, pas d'entree scrutees\r\n"
  "        mode L(ogic) :   Tout ou rien\r\n"
  "        mode A(nalog) :  Analogique \r\n"
  "        mode R(s485) :   jbus rs485\r\n"
  "        mode E(thernet) :jbus ip ethernet\r\n";
static void cmd_inputMode(BaseSequentialStream *chp, int argc,const char * const argv[]) 
{
  (void)argv;
  (void)chp;

  TEST_ARGC_EQ(1, usage_inputMode);
  switch (argv[0][0]) {
  case 'o':
  case 'O':
    setMode (IO_mode_Off);
    break;
    
  case 'l':
  case 'L':
    setMode (Mode_Logic);
    break;
    
  case 'a':
  case 'A':
    setMode (Mode_Analog_Pos);
    break;
    
  case 'e':
  case 'E':
    setMode (Mode_JbusEther);
    break;
    
  case 'r':
  case 'R':
    setMode (Mode_Jbus485);
    break;
  }
}



static int32_t get_stack_free (Thread *tp)
{
  int32_t index = 0;
  const uint8_t *maxRamAddr =  (uint8_t*) (0x20000000 + (128*1024));
  const int32_t internalStructSize = 80;
  
   unsigned long long *stkAdr =  (unsigned long long *) ((uint8_t *) tp  + internalStructSize); 
  //unsigned long long *stkAdr =  (unsigned long long *) tp; 

  while ((stkAdr[index] == 0x5555555555555555) && ( ((uint8_t *) &(stkAdr[index])) < maxRamAddr))
    index++;

  const int32_t freeBytes = index * sizeof(long long);
  return MAX(0, freeBytes - internalStructSize);
}



static const char *usage_threads =
  "CATEGORIE: mise au point\r\n"
  "permet de lister les threads chibios\r\n"
  "et d'afficher la place disponible dans la pile (colonne frestk)\r\n"
  "de chaque threads\r\n"
  "si la place disponible dans la pile est inferieure a 200 octets\r\n"
  "il y a risque de corruption generalisee de la memoire\r\n\r\n"
  "USAGE: threads";
static void cmd_threads(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  static const char *states[] = {THD_STATE_NAMES};
  Thread *tp;
  (void)argv;
  (void)argc;
  float totalTicks=0;
  float idleTicks=0;
  
  usage (usage_threads);

  chprintf (chp, "    addr    stack  frestk prio refs  state  time\r\n");
  tp = chRegFirstThread();
  do {
    chprintf (chp, "%.8lx %.8lx %6lu %4lu %4lu %9s %5lu %s\r\n",
	      (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
	      get_stack_free (tp),
	      (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
	      states[tp->p_state], (uint32_t)tp->p_time,
	      chRegGetThreadName(tp));
    totalTicks+= tp->p_time;
    if (strcmp (chRegGetThreadName(tp), "idle") == 0)
      idleTicks =  tp->p_time;
    tp = chRegNextThread(tp);
  } while (tp != NULL);

  const float idlePercent = (idleTicks*100.f)/totalTicks;
  const float cpuPercent = 100.f - idlePercent;
  chprintf (chp, "\r\ncpu load = %.2f\%\r\n", cpuPercent);
}

static const char *usage_readme =
  "CATEGORIE: documentation\r\n"
  "indique les etapes pour regler une carte nouvellement flashee\r\n\r\n"
  "USAGE:  readme";
static void cmd_readme(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  (void) argc;
  (void) argv;
  static bool_t cosmos=FALSE;

  chprintf (chp, 
	    "regler les parametres jbus (adresse jbus, adresse ip, param rs485)\r\n"
	    "et les sauver en eeprom\r\n"
	    "jbus ...\r\n"
	    "jbus save\r\n"
	    "\r\n"
	    "regler les pwmmin et pwm max et les sauver dans l'eeprom\r\n"
	    "pwm tune\r\n"
	    "save\r\n"
	    "\r\n"
	    "regler la position de repos et la sauver dans l'eeprom\r\n"
	    "pwm pos\r\n"
	    "save\r\n"
	    "\r\n"
	    "regler la date et l'heure de la rtc une fois la pile installe\r\n"
	    "rtc ...\r\n"
	    "tester les modes d'entree\r\n"
	    "mode ...\r\n"
	    "\r\n"
	    "aller se boire un cafe ...\r\n"
	    );

  if (cosmos == FALSE) {
    setSysLogMode (COSMOS);
    cosmos = TRUE;
  } else {
    setSysLogMode (SPLIT_PAGE);
     cosmos = FALSE;
  }

  

  /*  union { */
  /*   uint32_t val; */
  /*   uint8_t  buf [sizeof(uint32_t)] ; */
  /*  } ofst = {.val = 0x00025A00}; */

  /* char buffer [20] = {[0 ... 19] = 0xff}; */
  /* chsnprintf (buffer, sizeof(buffer),  */
  /* 	      "@m%s%c%c%c%c%c%c%c%c%c", "c.gci", 0, 0, 0, 0, 0, ofst.buf[3], ofst.buf[2], ofst.buf[1],  */
  /* 	      ofst.buf[0]); */

  /* for (uint32_t i=0; i< 20; i++) { */
  /*   DebugTrace ("buffer[%d] = 0x%x", i, buffer[i]); */
  /* } */


}


static const char *usage_help =
  "CATEGORIE: documentation\r\n"
  "affiche l'aide detaillee de la\r\n"
  "commande passee en argument\r\n\r\n"
  "USAGE: help [mem threads erase pwm save load wipe mode park jbus rtc gpio adc readme]";
static void cmd_help(BaseSequentialStream *chp, int argc,const char * const argv[]) {
  (void)chp;
  TEST_ARGC_EQ(1,usage_help);
  const char *cmd = argv[0];

  if (strcmp (cmd, "mem") == 0) {
    usage (usage_mem);
  } else if (strcmp (cmd, "threads") == 0) {
    usage (usage_threads);
  } else if (strcmp (cmd, "erase") == 0) {
    usage (usage_erase);
  } else if (strcmp (cmd, "pwm") == 0) {
    usage (usage_pwm);
  } else if (strcmp (cmd, "save") == 0) {
    usage (usage_save);
  } else if (strcmp (cmd, "load") == 0) {
    usage (usage_load);
  } else if (strcmp (cmd, "wipe") == 0) {
    usage (usage_wipe);
  } else if (strcmp (cmd, "mode") == 0) {
    usage (usage_inputMode);
  } else if (strcmp (cmd, "park") == 0) {
    usage (usage_park);
  } else if (strcmp (cmd, "jbus") == 0) {
    usage (usage_jbus);
  } else if (strcmp (cmd, "rtc") == 0) {
    usage (usage_rtc);
  } else if (strcmp (cmd, "gpio") == 0) {
    usage (usage_gpio);
  } else if (strcmp (cmd, "adc") == 0) {
    usage (usage_adc);
  } else if (strcmp (cmd, "help") == 0) {
    usage (usage_help);
  } else if (strcmp (cmd, "readme") == 0) {
    usage (usage_readme);
  } else {
    usage (usage_help);
  }
}






static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"erase", cmd_testFlashErase},
  {"pwm", cmd_pwm},
  {"save", cmd_saveEeprom},
  {"load", cmd_loadEeprom},
  {"wipe", cmd_wipeEeprom},
  {"mode", cmd_inputMode},
  {"park", cmd_park},
  {"jbus", cmd_jbus},
  {"rtc", cmd_rtc},
  {"calib", cmd_calib},
  {"gpio", cmd_testGpio},
  {"adc", cmd_testAdc},
  {"i2c", cmd_testI2c},
  {"lcd", cmd_testLcd},
  {"help", cmd_help},
  {"readme", cmd_readme},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};






void consoleLaunch (void)
{
  Thread *shelltp = NULL;

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * USBD1 : FS, USBD2 : HS
   */
  usbSerialInit(&SDU1, &USBDRIVER); 
  
  /*
   * Shell manager initialization.
   */
  shellInit();

   syslog (LOG_INFO, "USB NOT Rdy");
   while (TRUE) {
    if (!shelltp) {
      systime_t time=90;
      while (usbGetDriver()->state != USB_ACTIVE) {
	if (time != 100) {
	  time++;
	  chThdSleepMilliseconds(100);
	} else {
	  time=90;
	  usbSerialReset(&SDU1);
	}
      }

      syslog (LOG_INFO, "USB Ready");
      // activate driver, giovani workaround
      chnGetTimeout(&SDU1, TIME_IMMEDIATE);
      while (!isUsbConnected()) {
	chThdSleepMilliseconds(100);
      }
      
      syslog (LOG_INFO, "TTY connected");
      shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
    } else if (shelltp && (chThdTerminated(shelltp))) {
      chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
      shelltp = NULL;           /* Triggers spawning of a new shell.        */
    }
    chThdSleepMilliseconds(100);
  }

}

/*
  page prec [53 : passe au pwm precedent
  page suiv [54 : passe au pwm suivant
  Orig [49 : pwmmin
  Fin  [52 : pwmmax
  fleche haut [65 grande augmentation si pwmmax, diminution si pwmmin
  fleche bas  [66 grande diminution si pwmmax,  augmentation si pwmmin
  fleche droite [67   petite augmentation si pwmmax, diminution si pwmmin
  fleche gauche [68   petite diminution si pwmmax,  augmentation si pwmmin
 */
static void pwmTune_insert_char (uint8_t c, uint32_t mode)
{
  typedef enum  {VT_OUTSEQ, VT_ESC, VT_BRAQ} VtState;
  (void) mode;
  
  typedef struct {
    uint8_t idx;
    enum {PWM_MIN, PWM_MAX} minOrMax;
  } PwmToTune;

  
  static VtState vtState = VT_OUTSEQ;
  static PwmToTune pwm = {.idx = 0, .minOrMax = PWM_MIN};

  //  printScreen ("valChar is %d", c);
  switch (c) {
  case 27:
    if (vtState == VT_ESC) {
      chprintf (chp, "exit from pwm moder\r\n");
      vtState = VT_OUTSEQ;
      modeShell ();
      return;
    }
    vtState = VT_ESC;
    break;
    
  case '[':
    if (vtState == VT_ESC) {
      vtState = VT_BRAQ;
    }

  case 126:
    break;

  case 65:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;
    if (pwm.minOrMax == PWM_MIN) {
      servoSetPwmMin (pwm.idx, servoGetPwmMin (pwm.idx) - PWM_BIG_STEP);
      chprintf (chp, "pwm %d %s UP valeur: %d\r\n", pwm.idx, 
		  pwm.minOrMax == PWM_MIN ? "PWM_MIN" : "PWM_MAX",
		  servoGetPwmMin (pwm.idx));
    } else {
      servoSetPwmMax (pwm.idx, servoGetPwmMax (pwm.idx) + PWM_BIG_STEP);
       chprintf (chp, "pwm %d %s UP valeur: %d\r\n", pwm.idx, 
		  pwm.minOrMax == PWM_MIN ? "PWM_MIN" : "PWM_MAX",
		  servoGetPwmMax (pwm.idx));
   }
    break;

  case 66:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;
    if (pwm.minOrMax == PWM_MIN) {
      servoSetPwmMin (pwm.idx, servoGetPwmMin (pwm.idx) + PWM_BIG_STEP);
      chprintf (chp, "pwm %d %s DOWN valeur: %d\r\n", pwm.idx, 
		  pwm.minOrMax == PWM_MIN ? "PWM_MIN" : "PWM_MAX",
		  servoGetPwmMin (pwm.idx));
    } else {
      servoSetPwmMax (pwm.idx, servoGetPwmMax (pwm.idx) - PWM_BIG_STEP);
      chprintf (chp, "pwm %d %s DOWN valeur: %d\r\n", pwm.idx, 
		  pwm.minOrMax == PWM_MIN ? "PWM_MIN" : "PWM_MAX",
		  servoGetPwmMax (pwm.idx));
    }
    break;

  case 67:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;
    if (pwm.minOrMax == PWM_MIN) {
      servoSetPwmMin (pwm.idx, servoGetPwmMin (pwm.idx) - PWM_SMALL_STEP);
      chprintf (chp, "pwm %d %s up valeur: %d\r\n", pwm.idx, 
		  pwm.minOrMax == PWM_MIN ? "PWM_MIN" : "PWM_MAX",
		  servoGetPwmMin (pwm.idx));
    } else {
      servoSetPwmMax (pwm.idx, servoGetPwmMax (pwm.idx) + PWM_SMALL_STEP);
      chprintf (chp, "pwm %d %s up valeur: %d\r\n", pwm.idx, 
		  pwm.minOrMax == PWM_MIN ? "PWM_MIN" : "PWM_MAX",
		  servoGetPwmMax (pwm.idx));
    }
    break;

    
  case 68:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;
    if (pwm.minOrMax == PWM_MIN) {
      servoSetPwmMin (pwm.idx, servoGetPwmMin (pwm.idx) + PWM_SMALL_STEP);
      chprintf (chp, "pwm %d %s down valeur: %d\r\n", pwm.idx, 
		  pwm.minOrMax == PWM_MIN ? "PWM_MIN" : "PWM_MAX",
		  servoGetPwmMin (pwm.idx));
     } else {
      servoSetPwmMax (pwm.idx, servoGetPwmMax (pwm.idx) - PWM_SMALL_STEP);
      chprintf (chp, "pwm %d %s down valeur: %d\r\n", pwm.idx, 
		  pwm.minOrMax == PWM_MIN ? "PWM_MIN" : "PWM_MAX",
		  servoGetPwmMax (pwm.idx));
    }
    break;


  case 49:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;
    pwm.minOrMax = PWM_MIN;
    chprintf (chp, "pwm %d %s\r\n", pwm.idx, pwm.minOrMax == PWM_MIN ? "PWM_MIN" : "PWM_MAX");    
    break;

  case 52:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;
    pwm.minOrMax = PWM_MAX;
    chprintf (chp, "pwm %d %s\r\n", pwm.idx, pwm.minOrMax == PWM_MIN ? "PWM_MIN" : "PWM_MAX");    
    break;

  case 53:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;
    pwm.idx  = (pwm.idx == 0) ? SERVO_COUNT-1 : (pwm.idx-1) % SERVO_COUNT;
    chprintf (chp, "pwm %d %s\r\n", pwm.idx, pwm.minOrMax == PWM_MIN ? "PWM_MIN" : "PWM_MAX");    

  case 54:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;
    pwm.idx  = (pwm.idx+1) % SERVO_COUNT;
    chprintf (chp, "pwm %d %s\r\n", pwm.idx, pwm.minOrMax == PWM_MIN ? "PWM_MIN" : "PWM_MAX");    
    break;

    
 default:
   vtState = VT_OUTSEQ;
  } 

}


/*
  page prec [53 : passe au pwm precedent
  page suiv [54 : passe au pwm suivant
  fleche haut [65 grande augmentation 
  fleche bas  [66 grande diminution 
  fleche droite [67   petite augmentation 
  fleche gauche [68   petite diminution 
 */
static void pwmPos_insert_char (uint8_t c, uint32_t mode)
{
  typedef enum  {VT_OUTSEQ, VT_ESC, VT_BRAQ} VtState;
  (void) mode;
  
  typedef struct {
    uint8_t idx;
    enum {PWM_MIN, PWM_MAX} minOrMax;
  } PwmToTune;

  
  static VtState vtState = VT_OUTSEQ;
  static PwmToTune pwm = {.idx = 0, .minOrMax = PWM_MIN};

  switch (c) {
  case 27:
    if (vtState == VT_ESC) {
      chprintf (chp, "exit from pwm moder\r\n");
      vtState = VT_OUTSEQ;
      modeShell ();
      return;
    }
    vtState = VT_ESC;
    break;
    
  case '[':
    if (vtState == VT_ESC) {
      vtState = VT_BRAQ;
    }

  case 126:
    break;

  case 65:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;

    servoSetPos (pwm.idx, servoGetPos (pwm.idx) + POS_BIG_STEP);
    chprintf (chp, "pos %d UP valeur: %.2f\r\n", pwm.idx, 
	      servoGetPos (pwm.idx));
    break;

  case 66:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;

    servoSetPos (pwm.idx, servoGetPos (pwm.idx) - POS_BIG_STEP);
    chprintf (chp, "pos %d DOWN valeur: %.2f\r\n", pwm.idx, servoGetPos (pwm.idx));
    break;
    
  case 67:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;
    
    servoSetPos (pwm.idx, servoGetPos (pwm.idx) + POS_SMALL_STEP);
    chprintf (chp, "pos %d up valeur: %.2f\r\n", pwm.idx, servoGetPos (pwm.idx));
    break;

    
  case 68:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;
    
    servoSetPos (pwm.idx, servoGetPos (pwm.idx) - POS_SMALL_STEP);
    chprintf (chp, "pos %d down valeur: %.2f\r\n", pwm.idx, servoGetPos (pwm.idx));
    break;


  case 53:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;
    pwm.idx  = (pwm.idx == 0) ? SERVO_COUNT-1 : (pwm.idx-1) % SERVO_COUNT;
    chprintf (chp, "pwm %d\r\n", pwm.idx);    

  case 54:
    if (vtState != VT_BRAQ)
      break;
    vtState = VT_OUTSEQ;
    pwm.idx  = (pwm.idx+1) % SERVO_COUNT;
    chprintf (chp, "pwm %d\r\n", pwm.idx);    
    break;

    
 default:
   vtState = VT_OUTSEQ;
  } 

}

static void usage (const char * const msg)
{
  chprintf (chp, "%s\r\n", msg);
}

static void wrongArcCount(const uint8_t given, const uint8_t inf, const uint8_t sup, const char * const msg)
{
  if (inf == sup) {
    chprintf (chp, "%d args attendus au lieu de %d\r\n", sup, given);
  } else {
    chprintf (chp, "entre %d et %d args attendus au lieu de %d\r\n", inf, sup, given);
  }
  usage (msg);
}
