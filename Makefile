# MPLAB IDE generated this makefile for use with GNU make.
# Project: sensor2.mcp
# Date: Thu Feb 11 16:42:00 2010

AS = MPASMWIN.exe
CC = mcc18.exe
LD = mplink.exe
AR = mplib.exe
RM = rm

sensor2.cof : sensor2.o
	$(LD) /l"C:\MCC18\lib" /k"C:\MCC18\lkr" "18f45k20i.lkr" "sensor2.o" /u_CRUNTIME /u_DEBUG /u_DEBUGCODESTART=0x7dc0 /u_DEBUGCODELEN=0x240 /u_DEBUGDATASTART=0x5f4 /u_DEBUGDATALEN=0xc /z__MPLAB_BUILD=1 /z__MPLAB_DEBUG=1 /z__MPLAB_DEBUGGER_PK3=1 /z__ICD2RAM=1 /m"sensor2.map" /w /o"sensor2.cof"

sensor2.o : sensor2.c sensor2.h ../../MCC18/h/delays.h sensor2.c ../../MCC18/h/p18f45k20.h mma7455l.h ../../MCC18/h/p18cxxx.h
	$(CC) -p=18F45K20 /i"C:\MCC18\h" "sensor2.c" -fo="sensor2.o" -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1

clean : 
	$(RM) "sensor2.o" "sensor2.cof" "sensor2.hex" "sensor2.map"
