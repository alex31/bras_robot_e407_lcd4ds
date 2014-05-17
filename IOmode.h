#ifndef __IO_MODE_H__
#define __IO_MODE_H__
#include "ch.h"
#include "eeprom.h"

enum IO_modes {IO_mode_Off, 
	       Mode_Logic, Mode_Analog_Pos, Mode_Analog_Calib, Mode_Jbus485, Mode_JbusEther, 
	       Mode_Tuning, IO_mode_End};
enum IO_bits {Logic_none=0, Logic_in=0x1, Logic_out=0x1<<1, 
	      Analog_inCommand=1<<2, Analog_inCurrent=1<<3,  Analog_inServiPos=1<<4,
	      Jbus485_inout=1<<7, JbusEther_inout=1<<8, ServoPwm_out=1<<9};
enum IOThreads_bits {THD_Logic=1, THD_Analog=1<<1, THD_Jbus485=1<<2, THD_JbusEther=1<<3};

bool_t isActive (enum IO_bits bit);
void setMode (enum IO_modes mode);
enum IO_modes  getMode (void);
enum IO_bits getIoBits (enum IO_modes mode);
void setIoBit (enum IO_bits bit);
void clearIoBit (enum IO_bits bit);
enum IOThreads_bits getIoThreads (enum IO_bits iobit);
void manageThreadAccordingToMode (enum  IO_bits oldIoBits, enum IO_bits nexIoBits);
void IoModeInit (void);

#endif //  __IO_MODE_H__
