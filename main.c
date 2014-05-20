/*
  TODO

#        __          __   ___    _____    _   _   _____   _   _    _____
#        \ \        / /  / _ \  |  __ \  | \ | | |_   _| | \ | |  / ____|
#         \ \  /\  / /  | |_| | | |__) | |  \| |   | |   |  \| | | |  __
#          \ \/  \/ /   |  _  | |  _  /  | . ` |   | |   | . ` | | | |_ |
#           \  /\  /    | | | | | | \ \  | |\  |  _| |_  | |\  | | |__| |
#            \/  \/     |_| |_| |_|  \_\ |_| \_| |_____| |_| \_|  \_____|
  
                                                           _                
_|_  _    _  _|_     _|  _       _   _. ._ _|_  _      _|_  o | |  _      
 |_ (/_  _>   |_    (_| (/_     (_  (_| |   |_ (/_      |   | | | (/_     
                                                                          
                                     
|_  ._  _. ._   _  |_   _    _      
|_) |  (_| | | (_  | | (/_  (/_     

._ _   _   _| o _|_     |_   _. ._  _|    ._   _    _   _    _   _. o ._  _   
| | | (_) (_| |  |      | | (_| |  (_|    | | (/_  (_  (/_  _>  (_| | |  (/_  
                                                                              
 

  
  + DEBUG


  +INPUT
   ° test presence carte ecran, carte fille, carte robot
   ° test tension vbat, 3.3v
   ° calcul des vitesses à partir des positions
   ° detection de blocage
   ° calibration pour une table pwm <-> servo read
   ° inverse kinematic
   ° mode commande en pwm dilaté

  ° mecanisme d'abonnement sur les evenements boutons (vert, rouge, joystick)
    pour ce qu'on mettra dans le mode reglage
    le knob est un cas à part, il ne sert qu'à la selection du mode

  ° surintensité : gerer : 
    a: on part dans l'autre sens pour revenir sur une position libre
       (tableau en ram avec fenetre glissante de pos et conso toutes les 100ms)
    b: notification du pb syslog + son sur ecran 4d system
    c: si la surconso persiste : disable du servo
    d: on ignore les ordres sur ce servo jusqu'à un appui sur bouton vert "resume"
       qui execute les ordres sur le servo, et reenable le pwm si il avait été deconnecté

  ° retour de position : 
	° calibrer les 5 servos
        ° recherche automatique des min/max pwm

   +MODBUS
   ° FUTUR ?? : tcp event en fonction version modbus, autres fonctions modbus
                à implementer dans la lib freemodbus :  http://en.wikipedia.org/wiki/Modbus
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "stm32f4xx_rcc.h"
#ifdef WATCHDOG_ACTIVATED
#include "stm32f4xx_iwdg.h"
#endif
#include "ch.h"
#include "hal.h"
#include "printf.h"

#include "ttyConsole.h"
#include "stdutil.h"
#include "globalVar.h"
#include "servo_pwm.h"
#include "IOmode.h"
#include "lcdDisplay.h"
#include "analogicIO.h"
#include "userInput.h"
#include "errorLed.h"
#include "bitband.h"

static bool_t allSlaveBoardPlugged;


/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/


/*
 * Application entry point.
 */
int main(void) 
{
/*
#                __          __          _             _           _             __ _         
#                \ \        / /         | |           | |         | |           / _` |        
#                 \ \  /\  / /    __ _  | |_     ___  | |__     __| |    ___   | (_| |        
#                  \ \/  \/ /    / _` | | __|   / __| | '_ \   / _` |   / _ \   \__, |        
#                   \  /\  /    | (_| | \ |_   | (__  | | | | | (_| |  | (_) |   __/ |        
#                    \/  \/      \__,_|  \__|   \___| |_| |_|  \__,_|   \___/   |___/         
*/

#ifdef WATCHDOG_ACTIVATED
  bool_t rebootFromWatchdogReset = FALSE;
  if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)  {
    rebootFromWatchdogReset = TRUE;
  }
  RCC_ClearFlag();
  

  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  /* IWDG counter clock: LSI/32 */
  IWDG_SetPrescaler(IWDG_Prescaler_32);
  
  /* 2500 ms max to init all before watchdog fire */
  IWDG_SetReload(LsiFreq/10);
  
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();

  /* Stop IWDG timer when core is halted during debug or  firmware upload */
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#endif

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  chHeapInit(&ccmHeap, (void *) ccmHeapBuffer, sizeof (ccmHeapBuffer));

  errorLedInit();

  allSlaveBoardPlugged = isDaughterBoardPlugged() && 
    isLcdBoardPlugged() && isArmBoardPlugged ();

  
  /*
   * Creates the LCD Display thread.
   */
  static WORKING_AREA(waLCDThd, 3*1024);
  chThdCreateStatic(waLCDThd, sizeof(waLCDThd), NORMALPRIO-2, thdLcdDisplay, 0);
  chThdSleepMilliseconds(100);

   if (allSlaveBoardPlugged) {
     IoModeInit ();
   }
  
  if (isLcdBoardPlugged()) {
    syslog (LOG_INFO, "Fw %s %s", __DATE__, __TIME__);
    syslog (LOG_INFO, "Free Flash %d", eepromStatus());
    syslog (LOG_INFO, "Free Ram %d", chCoreStatus());
    if (!isDaughterBoardPlugged()) {
      syslog (LOG_FATAL, "DaughterBoard not plugged");
    } else {
      syslog (LOG_INFO, "DaughterBoard OK");
    }
    if (!isArmBoardPlugged()) {
      syslog (LOG_FATAL, "ArmBoard not plugged");
    } else {
      syslog (LOG_INFO, "ArmBoard OK");
    }


#ifdef WATCHDOG_ACTIVATED
    if (rebootFromWatchdogReset) {
      syslog (LOG_ERROR, "WATCHDOG FIRE");
    }
#endif
  }

  if (allSlaveBoardPlugged) {
    /* calibrate current adc sensors before servo motor are drived by pwm*/
    setMode (Mode_Analog_Calib);
    chThdSleepMilliseconds(100); // leave time for filter to converge
    analogCalibrateCurrentIntensity ();
    syslog (LOG_INFO, "adc calib.");
  }
  
  /*
   * initialize the PWM (5 pin)
   */
  
  if (allSlaveBoardPlugged) {
    initAllServos ();
    /*
     * Creates the PWM thread.
     */
    static WORKING_AREA(waPwmThd, 512);
    chThdCreateStatic(waPwmThd, sizeof(waPwmThd), NORMALPRIO, thdPwm, 0);
    
    syslog (LOG_INFO, "pwm lauched");
  }
  
  /*
   * Creates the input mode (via  buttons, knob and joysticks) thread.
   */
  if (isLcdBoardPlugged ()) {
    launchInputKnob ();
    launchInputJoystick ();
  }
  
  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  
  if (isLcdBoardPlugged()) {
    consoleLaunch();
  } else {
    goto justBlink;  
  }
  
 justBlink:
  {
    lightFlash (ERROR_H407_LED, 4, 1000, 500, 1, ERROR_LED_TIME_INFINITE);
    lightFlash (ERROR_LED_RED, 4, 1000, 500, 1, ERROR_LED_TIME_INFINITE);
    lightFlash (ERROR_GREEN_LED, 4, 1000, 500, 1, ERROR_LED_TIME_INFINITE);
    do {
      // test de la presence d'un jumper entre gpioF8 et gpioF9 (Pin11 et Pin12 du connecteur F)
      bb_palSetPad (GPIOF, GPIOF_NAKED_TEST_OUT);
      chThdSleepMilliseconds(1);
      jumperDebugPresent= bb_palReadPad (GPIOF, GPIOF_NAKED_TEST_IN);

      bb_palClearPad (GPIOF, GPIOF_NAKED_TEST_OUT);
      chThdSleepMilliseconds(1);
      jumperDebugPresent &= !bb_palReadPad (GPIOF, GPIOF_NAKED_TEST_IN);

      chThdSleepMilliseconds(1000);
    } while (!jumperDebugPresent);
  }

  lightFlash (ERROR_H407_LED, 4, 2000, 200, 2, ERROR_LED_TIME_INFINITE);
  syslog (LOG_ERROR, "**********");
  syslog (LOG_ERROR, "Mode DEBUG");
  syslog (LOG_ERROR, "Carte Nue");
  syslog (LOG_ERROR, "Selectionnee");
  syslog (LOG_ERROR, "Par jumper sur");
  syslog (LOG_ERROR, "D11 D12");
  syslog (LOG_ERROR, "**********");
  initAllServos ();
  /*
   * Creates the PWM thread.
   */
  static WORKING_AREA(waPwmThd, 512);
  chThdCreateStatic(waPwmThd, sizeof(waPwmThd), NORMALPRIO, thdPwm, 0);
    
  syslog (LOG_INFO, "pwm lauched");
  // mode debug carte nue : on lance juste la console
  consoleLaunch();
  
  
}

void  port_halt (void)
{
  volatile unsigned int count;
   while (1) {
     bb_palClearPad (GPIOF, GPIOF_LED_RED);
     bb_palSetPad (GPIOG, GPIOG_LED_GREEN);
     count = 2*1000*1000;
     while (--count) {};
     bb_palSetPad (GPIOF, GPIOF_LED_RED);
     bb_palClearPad (GPIOG, GPIOG_LED_GREEN);
     count = 2*1000*1000;
     while (--count) {};
   }
}

void generalKernelErrorCB (void)
{
  DebugTrace ("Kernel panic : \r\n%s",
             dbg_panic_msg); 
  syslog (LOG_FATAL, "Kernel panic");
  port_halt ();
}






