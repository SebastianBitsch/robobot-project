#*********************************************************************
# This is the makefile for the Adafruit SSD1306 OLED library driver
#
#	02/18/2013 	Charles-Henri Hallard (http://hallard.me)
#							Modified for compiling and use on Raspberry ArduiPi Board
#							LCD size and connection are now passed as arguments on 
#							the command line (no more #define on compilation needed)
#							ArduiPi project documentation http://hallard.me/arduipi
#
# 08/26/2015    Lorenzo Delana (lorenzo.delana@gmail.com)
#                                                       added bananapi specific CCFLAGS and conditional macro BANANPI
#
# *********************************************************************

# Makefile itself dir
#ROOT_DIR:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

# hw platform as from autogen.sh choose
#HWPLAT:=$(shell cat $(ROOT_DIR)/../hwplatform)

# sets CCFLAGS hw platform dependant
# ifeq ($(HWPLAT),BananaPI)
#         CCFLAGS=-Wall -Ofast -mfpu=vfpv4 -mfloat-abi=hard -march=armv7 -mtune=cortex-a7 -DBANANAPI
# else # fallback to raspberry
        # The recommended compiler flags for the Raspberry Pi
#        CCFLAGS=-Wall -Ofast -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s
# endif

CPU := $(shell uname -m)
$(info $$CPU is [${CPU}])

ifeq ($(CPU), armv7l)
  CCFLAGS=-Wall -Ofast -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s -DRASPBERRY_PI
else
  CCFLAGS=-Wall -O0 -DNOT_RASPBERRY_PI -Wno-unused-variable -Wno-pointer-to-int-cast -Wno-return-type
endif
$(info $$CCFLAGS is [${CCFLAGS}])



prefix := /usr/local

# define all programs
PROGRAMS = oled_disp_ip
SOURCES = ${PROGRAMS:=.cpp} 
OBJS = Adafruit_GFX.o ArduiPi_OLED.o Wrapper.o bcm2835.o 


all: ${PROGRAMS} ${OBJS} ArduiPi_OLED_lib.h

# Make the library
# ArduiPi_OLED: ${OBJS}
# 	g++ -shared -Wl,-soname,$@.so.1 ${CCFLAGS}  -o ${LIBNAME} $^

# Library parts (use -fno-rtti flag to avoid link problem)
ArduiPi_OLED.o: ArduiPi_OLED.cpp
	g++   ${CCFLAGS} -c $^
# g++ -Wall -fPIC -fno-rtti ${CCFLAGS} -c $^

Adafruit_GFX.o: Adafruit_GFX.cpp
	g++  ${CCFLAGS} -c $^
# g++ -Wall -fPIC -fno-rtti ${CCFLAGS} -c $^

bcm2835.o: bcm2835.c
	gcc  ${CCFLAGS} -c $^
#gcc -Wall -fPIC ${CCFLAGS} -c $^

Wrapper.o: Wrapper.cpp
	gcc  ${CCFLAGS} -c $^


${PROGRAMS}: ${SOURCES} ${OBJS}
	g++ ${CCFLAGS}  $@.cpp -o $@ $(OBJS) -lreadline

clean:
	rm -rf $(PROGRAMS) $(OBJS)

install: all
	test -d $(prefix) || mkdir $(prefix)
	test -d $(prefix)/bin || mkdir $(prefix)/bin
	for prog in $(PROGRAMS); do \
	  install -m 0755 $$prog $(prefix)/bin; \
	done

.PHONY: install


