CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
AS = arm-none-eabi-as
AR = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy

CFLAGS += -std=c99 -Wall -Wextra -Wpedantic

CFLAGS += -I/usr/arm-none-eabi/include/

ifeq ($(DEBUG),1)
CFLAGS += -Og -ggdb
else
CFLAGS += -O2
endif

ifdef TARGET
include targets/$(TARGET)
endif

help:
	$(info Run `make TARGET=$$TARGET` where $$TARGET is one of:)
	@find targets -type f ! -name '_*' -printf "%f\n"

clean:
	rm -f *.o *.a

.PHONY: help clean
