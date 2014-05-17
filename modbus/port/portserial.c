/*
 * FreeModbus Libary: Atmel AT91SAM3S Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: portserial.c,v 1.1 2010/06/06 13:07:20 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>
#include <ch.h>
#include <hal.h>
#include "stdutil.h"
#include "IOmode.h"
#include "lcdDisplay.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "modbus/port/port.h"
#include "modbus/include/mb.h"
#include "modbus/include/mbport.h"

/* ----------------------- Defines ------------------------------------------*/

#define USART_INVALID_PORT      ( 0xFF )
#define USART_NOT_RE_IDX        ( 3 )
#define USART_DE_IDX            ( 4 )
#define USART_IDX_LAST          ( 1 ) // only one usart

/* ----------------------- Static variables ---------------------------------*/
static void txDriverHasRead(UARTDriver *uartp) ;
static void txBufferEmpty(UARTDriver *uartp) ;
static void rxErr(UARTDriver *uartp, uartflags_t e);
static void rxChar(UARTDriver *uartp, uint16_t c);
static void rxEnd(UARTDriver *uartp);

static UCHAR    ucUsedPort = USART_INVALID_PORT;
static UCHAR    oneByteAccum = 0; // should we use a circular buffer ?

#ifdef DEBUG_MB
static CHAR lastCharReceived;

CHAR getLastCharReceived (void)
{
  return lastCharReceived;
}
#endif  // DEBUG_MB

void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
  USART_TypeDef *u = UARTDRIVER.usart;

  if( xRxEnable )  {
    //palSetPadMode (BOARD_SERIAL_PORT, BOARD_SERIAL_RX, BOARD_SERIAL_ALTERNATE);
    (u->CR1) |=  USART_CR1_RXNEIE;
  }  else {
    (u->CR1) &= ~USART_CR1_RXNEIE;
  }

  if( xTxEnable )  {
    //palSetPadMode (BOARD_SERIAL_PORT, BOARD_SERIAL_TX, BOARD_SERIAL_ALTERNATE);
    (u->CR1) |= USART_CR1_TCIE;
    pxMBFrameCBTransmitterEmpty ();
  } else {
    (u->CR1) &= ~USART_CR1_TCIE;
  }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity, UCHAR nbBitsStop)
{
  
  BOOL            bStatus = FALSE;

  static UARTConfig uartCfg = {
    txDriverHasRead,
    txBufferEmpty,
    rxEnd,
    rxChar,
    rxErr,
    9600,
    0,
    USART_CR2_LINEN,
    0 // no rts cts
  };



  if (ucPORT <= USART_IDX_LAST)  {
    bStatus = TRUE;
    uartCfg.speed = ulBaudRate;
    // debug with gnuscreen or modpoll on pc cannot set mode 2 bits stop
    // so be care to only use 1 bit stop when connection a pc
    if (nbBitsStop == 2)
      uartCfg.cr2 |= USART_CR2_STOP_1; // 2 stop bit 

    switch ( eParity )        {
    case MB_PAR_NONE:
      break;
    case MB_PAR_ODD:
      uartCfg.cr1 |= (USART_CR1_PCE|USART_CR1_PS); // parity odd 
      break;
    case MB_PAR_EVEN:
      uartCfg.cr1 |= USART_CR1_PCE; // parity even 
      break; 
    default:
      bStatus = FALSE;
      break;
    }

    switch ( ucDataBits )  {
    case 8:
      if (eParity !=  MB_PAR_NONE) {
	uartCfg.cr1 |= USART_CR1_M; // 8 bit + parity : we should put usart in 9 bits mode
      }
      break;
    case 7:
      break;
    default:
      bStatus = FALSE;
    }

    if (bStatus == TRUE)  {
      uartStart(&UARTDRIVER, &uartCfg);
    }
  }
  ucUsedPort = ucPORT;
  return bStatus;
}

void
vMBPortSerialClose( void )
{
  if (ucUsedPort != USART_INVALID_PORT)   {
    uartStop (&UARTDRIVER);
    ucUsedPort = USART_INVALID_PORT;
  }
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
  CHAR toSend = ucByte;
  //  palClearPad (BOARD_LED2_P, BOARD_LED2);

  if (bMBPortIsWithinException() == TRUE) {
    uartStartSendI (&UARTDRIVER, 1, &toSend);
  } else {
    uartStartSend (&UARTDRIVER, 1, &toSend);
  }
  return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
  *pucByte = oneByteAccum;
  return TRUE;
}

static void txDriverHasRead(UARTDriver *uartp) 
{
  (void) uartp;

}

static void txBufferEmpty(UARTDriver *uartp) 
{
  (void) uartp;

  chSysLockFromIsr(); {
    vMBPortSetWithinException (TRUE);
    //    palSetPad (BOARD_LED2_P, BOARD_LED2);


#ifndef DEBUG_MB
    if (pxMBFrameCBTransmitterEmpty () == TRUE)
      rescheduleJbus485FromIsr ();
#else
      static char charCount = 'a';
    if (charCount < 'z') {
      xMBPortSerialPutByte (charCount++);
    } else {
      charCount = 'a';
      vMBPortSerialEnable (FALSE, FALSE);
    }
#endif
    vMBPortSetWithinException (FALSE);
  } chSysUnlockFromIsr();
  
}

static void rxErr(UARTDriver *uartp, uartflags_t e)
{
  (void) uartp;
  (void) e;
  USART_TypeDef *u = UARTDRIVER.usart;

  chSysLockFromIsr(); 
  if (e & USART_SR_PE) {
    syslogErrorFromISR ("parity err");
  } else if (e & USART_SR_FE) {
    syslogErrorFromISR ("framing err");
  } else if (e & USART_SR_NE) {
    syslogErrorFromISR ("noise err");
  } else if (e & USART_SR_ORE) {
    syslogErrorFromISR ("overrun err");
  } else if (e & USART_SR_IDLE) {
    syslogErrorFromISR ("idle line err");
  } 
  
  (u->SR) &= ~(USART_SR_RXNE|USART_SR_ORE);
  chSysUnlockFromIsr();
}

static void rxChar(UARTDriver *uartp, uint16_t c)
{
  (void) uartp;
  (void) c;

  oneByteAccum = (UCHAR) c;

  vMBPortSetWithinException (TRUE);
  chSysLockFromIsr(); {
#ifdef DEBUG_MB
    xMBPortSerialGetByte (&lastCharReceived);
#else
    pxMBFrameCBByteReceived ();
    rescheduleJbus485FromIsr ();
#endif
    
  } chSysUnlockFromIsr();
  vMBPortSetWithinException (FALSE);
}

static void rxEnd(UARTDriver *uartp)
{  
  (void) uartp;
  //  palSetPad(BOARD_LED2_P, BOARD_LED2);
}
