# List of the ChibiOS/RT Cortex-M4 STM32 port files.
PORTSRC = $(CHIBIOS)/os/ports/GCC/ARMCMx/crt0.c \
          $(CHIBIOS)/os/ports/GCC/ARMCMx/STM32F4xx/vectors.c \
          ${CHIBIOS}/os/ports/GCC/ARMCMx/chcore.c \
          ${CHIBIOS}/os/ports/GCC/ARMCMx/chcore_v7m.c \
          ${CHIBIOS}/os/ports/common/ARMCMx/nvic.c

# .s extension is required by chibios example makefiles
# .S is more appropriate
PORTASM = ${CHIBIOS}/os/ports/GCC/ARMCMx/chcoreasm_v7m.s

PORTINC = ${CHIBIOS}/os/ports/common/ARMCMx/CMSIS/include \
          ${CHIBIOS}/os/ports/common/ARMCMx \
          ${CHIBIOS}/os/ports/GCC/ARMCMx \
          ${CHIBIOS}/os/ports/GCC/ARMCMx/STM32F4xx

PORTLD  = ${CHIBIOS}/os/ports/GCC/ARMCMx/STM32F4xx/ld
