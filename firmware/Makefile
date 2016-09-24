
#Compiler
SDCC ?= sdcc

NRFPROG ?= ../nrfProg/nrfProg

VERIFIED ?= 0

CFLAGS = -Iinc/ --model-large --std-sdcc99
LDFLAGS = --xram-loc 0x8000 --xram-size 2048 --model-large

VPATH += src/ bin/

OBJS = main.rel radio.rel usb.rel usbDescriptor.rel led.rel utils.rel hal_uart.rel

# jason.kim 2016.9.24 Support HID keyboard
PPM_KEYBOARD = 1
# PPM_JOYSTICK = 1

ifdef PPM_JOYSTICK
ifneq ($(PPM_JOYSTICK), 0)
  OBJS += ppm.rel
  CFLAGS += -DPPM_JOYSTICK
endif
endif

# jason.kim 2016.9.24 Support HID keyboard
ifdef PPM_KEYBOARD
ifneq ($(PPM_KEYBOARD), 0)
  CFLAGS += -DPPM_KEYBOARD
endif
endif

ifdef CRPA
  CFLAGS += -DCRPA
endif

.PHONY: all cri

all: cradio.bin
ifdef CRPA
	@echo Crazyradio PA build
else
	@echo Original Crazyradio build
endif

cri: cradio.cri

# Main targets
cradio.bin: $(OBJS)
	$(SDCC) $(LDFLAGS) $(OBJS:%=bin/%) -o bin/cradio.ihx
	objcopy -I ihex bin/cradio.ihx -O binary bin/cradio.bin

#Crazyradio firmware image
bin/cradio.cri: bin/cradio.bin
	python2 ../usbtools/generateCri.py bin/cradio.bin bin/cradio.cri $(VERIFIED)

clean:
	rm -f bin/*

mrproper: clean
	rm -f src/*~ inc/*~ *~

# CPU control targets
flash: all
	$(NRFPROG) write cradio.bin

halt:
	$(NRFPROG) holdreset

reset:
	$(NRFPROG) reset

# General targets
%.rel: %.c
	$(SDCC) $(CFLAGS) -c $< -o bin/$@

bin/:
	mkdir -p bin
