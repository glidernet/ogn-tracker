# Tool chain downloaded from: https://launchpad.net/gcc-arm-embedded
# unpacked to a directory: TPATH

TPATH = ../gcc-arm-none-eabi-4.8/bin
TCHAIN = arm-none-eabi

CC      = $(TPATH)/$(TCHAIN)-gcc
CPP     = $(TPATH)/$(TCHAIN)-g++
OBJCOPY = $(TPATH)/$(TCHAIN)-objcopy
OBJDUMP = $(TPATH)/$(TCHAIN)-objdump
ARCH    = $(TPATH)/$(TCHAIN)-ar

CC_SRC     = main.c
CC_SRC    += spi.c
CC_SRC    += spirit1.c
CC_SRC    += stm32l1xx_it.c
CC_SRC    += options.c
CC_SRC    += usart.c
CC_SRC    += console.c
CC_SRC    += commands.c
CC_SRC    += cir_buf.c 
CC_SRC    += cmsis_lib/Source/stm32l1xx_usart.c
CC_SRC    += cmsis_lib/Source/stm32l1xx_gpio.c
CC_SRC    += cmsis_lib/Source/stm32l1xx_flash.c
CC_SRC    += cmsis_lib/Source/stm32l1xx_rcc.c
CC_SRC    += cmsis_lib/Source/stm32l1xx_dma.c
CC_SRC    += cmsis_lib/Source/stm32l1xx_spi.c
CC_SRC    += cmsis_lib/Source/misc.c
CC_SRC    += cmsis_boot/Startup/startup_stm32l1xx_hd.c
CC_SRC    += cmsis_boot/system_stm32l1xx.c
CC_SRC    += free_rtos_cli/FreeRTOS_CLI.c
CC_SRC    += free_rtos/list.c
CC_SRC    += free_rtos/heap_2.c
CC_SRC    += free_rtos/port.c
CC_SRC    += free_rtos/event_groups.c
CC_SRC    += free_rtos/croutine.c
CC_SRC    += free_rtos/tasks.c
CC_SRC    += free_rtos/timers.c
CC_SRC    += free_rtos/queue.c
CC_SRC    += spirit1_dk/src/SPIRIT_General.c
CC_SRC    += spirit1_dk/src/SPIRIT_Radio.c
CC_SRC    += spirit1_dk/src/SPIRIT_Commands.c
CC_SRC    += spirit1_dk/src/SPIRIT_Management.c
CC_SRC    += spirit1_dk/src/SPIRIT_Calibration.c
CC_SRC    += spirit1_dk/src/SPIRIT_Types.c

H_SRC      = main.h
H_SRC     += options.h
H_SRC     += console.h
H_SRC     += commands.h
H_SRC     += cir_buf.h
H_SRC     += usart.h
H_SRC     += spirit1.h
H_SRC     += spi.h
H_SRC     += messages.h
H_SRC     += gps.h
H_SRC     += FreeRTOSConfig.h
H_SRC     += ogn.h
H_SRC     += ldpc.h
H_SRC     += bitcount.h
H_SRC     += nmea.h

CPP_SRC    = gps.cpp

DEFS       = -DSTM32L1XX_XL -DUSE_STDPERIPH_DRIVER

INCDIR     = -I.
INCDIR    += -Ifree_rtos/include -Ifree_rtos_cli
INCDIR    += -Icmsis -Icmsis_boot -Icmsis_lib/Include
INCDIR    += -Ispirit1_dk/inc

LDSCRIPT   = arm-gcc-link.ld

CC_OBJ     = $(CC_SRC:.c=.o)
CPP_OBJ    = $(CPP_SRC:.cpp=.o)

CC_OPT     = -mcpu=cortex-m3 -mthumb -Wall -O3 -g -ffunction-sections -std=c99
CPP_OPT    = -mcpu=cortex-m3 -mthumb -Wall -O3 -g -ffunction-sections

LNK_OPT    = -mcpu=cortex-m3 -mthumb -nostartfiles -O2 -g -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -Wl,-Map=main.map -T$(LDSCRIPT)

all:	main.hex main.bin main.dmp

$(CC_OBJ) : %.o : %.c makefile $(H_SRC)
	$(CC)  -c $(CC_OPT)  $(INCDIR) $(DEFS) $< -o $@

$(CPP_OBJ) : %.o : %.cc makefile $(H_SRC)
	$(CPP) -c $(CPP_OPT) $(INCDIR) $(DEFS) $< -o $@

main.elf:	$(CC_OBJ) $(CPP_OBJ) makefile
	$(CPP) $(LNK_OPT) -o $@ $(CC_OBJ) $(CPP_OBJ)

main.hex:	main.elf
	$(OBJCOPY) -O ihex $< $@

main.bin:	main.elf
	$(OBJCOPY) -O binary $< $@

main.dmp:	main.elf
	$(OBJDUMP) -d -S $< > $@

clean:
	rm -f main.elf main.map main.hex main.bin main.dmp $(CC_OBJ) $(CPP_OBJ) *.o

arch:
	tar cvzf OGN_Proto.tgz makefile *.h *.c *.cc *.ld free_rtos free_rtos_cli cmsis cmsis_boot cmsis_lib
