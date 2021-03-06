# MPLAB IDE generated this makefile for use with GNU make.
# Project: test01.mcp
# Date: Tue Apr 22 15:01:24 2014

AS = pic30-as.exe
CC = pic30-gcc.exe
LD = pic30-ld.exe
AR = pic30-ar.exe
HX = pic30-bin2hex.exe
RM = rm

test01.hex : test01.cof
	$(HX) "test01.cof"

test01.cof : test.o uart.o pwm.o i2c.o
	$(CC) -mcpu=30F4011 "test.o" "uart.o" "pwm.o" "i2c.o" -o"test01.cof" -Wl,-L"C:\Program Files\Microchip\MPLAB C30\lib",--script="..\..\..\..\..\..\Program Files\Microchip\MPLAB C30\support\dsPIC30F\gld\p30f4011.gld",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,-Map="test01.map",--report-mem

test.o : i2c.h uart.h ../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/dsPIC30F/h/p30f4011.h ../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/dsPIC30F/h/p30fxxxx.h ../../../../../../program\ files/microchip/mplab\ c30/support/peripheral_30F_24H_33F/adc10.h ../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/dsPIC30F/h/p30f4011.h ../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/dsPIC30F/h/p30fxxxx.h test.c
	$(CC) -mcpu=30F4011 -x c -c "test.c" -o"test.o" -I"C:\Program Files\Microchip\MPLAB C30\support\dsPIC30F\h" -D__DEBUG -g -Wall

uart.o : ../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/dsPIC30F/h/p30f4011.h ../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/dsPIC30F/h/p30fxxxx.h uart.h uart.c
	$(CC) -mcpu=30F4011 -x c -c "uart.c" -o"uart.o" -I"C:\Program Files\Microchip\MPLAB C30\support\dsPIC30F\h" -D__DEBUG -g -Wall

pwm.o : ../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/dsPIC30F/h/p30f4011.h ../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/dsPIC30F/h/p30fxxxx.h pwm.h pwm.c
	$(CC) -mcpu=30F4011 -x c -c "pwm.c" -o"pwm.o" -I"C:\Program Files\Microchip\MPLAB C30\support\dsPIC30F\h" -D__DEBUG -g -Wall

i2c.o : ../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/dsPIC30F/h/p30f4011.h ../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/dsPIC30F/h/p30fxxxx.h i2c.h i2c.c
	$(CC) -mcpu=30F4011 -x c -c "i2c.c" -o"i2c.o" -I"C:\Program Files\Microchip\MPLAB C30\support\dsPIC30F\h" -D__DEBUG -g -Wall

clean : 
	$(RM) "test.o" "uart.o" "pwm.o" "i2c.o" "test01.cof" "test01.hex"

