##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
# -falign-functions=16
# -fomit-frame-pointer 

#DISPLAY=OLED
DISPLAY=PICASO4


# Compiler options here.

ifeq ($(USE_OPT),)
  USE_OPT = -std=gnu11 -O0 -ggdb3 -fno-inline  -fshort-enums \
          -falign-functions=16 -fomit-frame-pointer \
          -Werror -Wno-error=unused-variable -Wno-error=format \
          -Wno-error=unused-function \
          -ftrack-macro-expansion=2 -Wno-error=strict-overflow -Wstrict-overflow=5
endif

ifeq ($(USE_OPT),)
  USE_OPT = -std=gnu11 -Ofast -flto -ggdb3 -fshort-enums \
          -falign-functions=16 -fomit-frame-pointer \
          -Werror -Wno-error=unused-variable -Wno-error=format \
          -Wno-error=unused-function \
          -ftrack-macro-expansion=2 -Wno-error=strict-overflow -Wstrict-overflow=1 \
	  -u prvGetRegistersFromStack
endif



# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Enables the use of FPU on Cortex-M4.
# Enable this if you really want to use the STM FWLib.
ifeq ($(USE_FPU),)
  USE_FPU = yes
endif

# Enable this if you really want to use the STM FWLib.
ifeq ($(USE_FWLIB),)
  USE_FWLIB = no
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = ch

# Imported source files and paths
CHIBIOS = ./ChibiOS
STMSRC = ./stm
VARIOUS = ./various

include ./chibios/boards/OLIMEX_STM32_E407/board.mk
include $(CHIBIOS)/os/hal/platforms/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/ports/GCC/ARMCMx/STM32F4xx/port.mk
include $(CHIBIOS)/os/kernel/kernel.mk
include $(CHIBIOS)/os/various/lwip_bindings/lwip.mk
include $(CHIBIOS)/test/test.mk
include modbus/modbus.mk

# Define linker script file here
LDSCRIPT= ./chibios/os/ports/GCC/ARMCMx/STM32F4xx/ld/STM32F407xG_S11_EEPROM.ld


# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(TESTSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(LWSRC) \
       $(MODBUSSRC) \
       $(STMSRC)/stm32f4xx_iwdg.c \
       $(STMSRC)/stm32f4xx_rcc.c \
       $(STMSRC)/stm32f4xx_flash.c \
       $(CHIBIOS)/os/various/evtimer.c \
       $(CHIBIOS)/os/various/chrtclib.c \
       $(VARIOUS)/printf.c \
       $(VARIOUS)/stdutil.c \
       $(VARIOUS)/lcdDisplay.c \
       $(VARIOUS)/rtcAccess.c \
       $(VARIOUS)/i2cMaster.c \
       microrl/microrlShell.c \
       microrl/microrl.c \
       globalVar.c \
       usb_serial.c \
       eeprom.c \
       servo_pwm.c \
       logicLevelIO.c \
       analogicIO.c \
       IOmode.c \
       jbus_common.c \
       jbus485.c \
       jbusEther.c \
       ttyConsole.c \
       userInput.c \
       errorLed.c \
       calibration.c \
       main.c

ifeq ($(DISPLAY), PICASO4)
  CSRC += picaso4Display.c
endif

ifeq ($(DISPLAY), OLED)
  CSRC += oledDisplay.c
endif

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(PORTASM)

INCDIR = $(PORTINC) $(KERNINC) $(TESTINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) $(LWINC) \
         $(VARIOUS) $(CHIBIOS)/os/various $(STMSRC)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -D_DEBUG=1
# -DTRACE
DDEFS =  -DDEBUG -D$(DISPLAY) -D__COVERITY__

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR = $(MODBUSINC)

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS = -lm

#
# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user defines
##############################################################################

ifeq ($(USE_FPU),yes)
  USE_OPT += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant
  DDEFS += -DCORTEX_USE_FPU=TRUE
else
  DDEFS += -DCORTEX_USE_FPU=FALSE
endif


ifeq ($(USE_FWLIB),yes)
  include $(CHIBIOS)/ext/stm32lib/stm32lib.mk
  CSRC += $(STM32SRC)
  INCDIR += $(STM32INC)
  USE_OPT += -DUSE_STDPERIPH_DRIVER
endif


default : all


lcdDisplay.c : cosmosOffsetArray.h


include ./chibios/os/ports/GCC/ARMCMx/rules.mk
