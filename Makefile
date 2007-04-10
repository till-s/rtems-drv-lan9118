TOP=..

ifeq ("$(wildcard $(TOP)/configure/CONFIG)xx","xx")
# TOP/configure/CONFIG doesn't exist. Use RTEMS Makefile
include Makefile.rtems
else
# we're inside EPICS

include $(TOP)/configure/CONFIG

CROSS_COMPILER_TARGET_ARCHS=RTEMS-uC5282 RTEMS-beatnik
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library
LIBRARY_HOST=udpCommSupport

INC += padProto.h udpComm.h padStream.h


#Indicate that we want to use 'normal' socket implementation
USR_CPPFLAGS += -DBSDSOCKET

# install devXxxSoft.dbd into <top>/dbd
#DBD += bpmPadSupport.dbd

# The following are compiled and added to the Support library
udpCommSupport_SRCS += padProto.c
udpCommSupport_SRCS += udpCommBSD.c

#===========================

include $(TOP)/configure/RULES

#----------------------------------------
#  ADD RULES AFTER THIS LINE

endif 
