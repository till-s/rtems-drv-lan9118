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

PROD_HOST=padProtoHost

USE_SDDS=YES

SDDS_YES_SRCS=sddsrd.c
# a hack because my SDDS build used another BASE version
# and the paranoia check wouldn't let me use it
ifeq (hack,)
SDDS_YES_LIBS=SDDS1c
else
SDDS_YES_LDFLAGS=-L/home/till/epics/extensions/lib/$(T_A)
USR_LDFLAGS+=$(SDDS_$(USE_SDDS)_LDFLAGS)
SDDS_YES_LIBS=SDDS1c
endif
SDDS_YES_CPPFLAGS=-DUSE_SDDS

padProtoHost_SRCS=padProtoHost.c bpmsim.c hostStream.c
padProtoHost_SRCS+=$(SDDS_$(USE_SDDS)_SRCS)
padProtoHost_LIBS=udpCommSupport
padProtoHost_LDLIBS+=$(SDDS_$(USE_SDDS)_LIBS)

INC += padProto.h udpComm.h padStream.h


#Indicate that we want to use 'normal' socket implementation
USR_CPPFLAGS += -DBSDSOCKET $(SDDS_$(USE_SDDS)_CPPFLAGS)

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
