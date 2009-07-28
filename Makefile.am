#  $Id$
AUTOMAKE_OPTIONS=foreign

include $(top_srcdir)/rtems-pre.am

EXEEXT=$(OBJEXEEXT)

liblanIpBasic_a_SOURCES        = lanIpBasicTest.c drvUdpSock.c hwtmr.h

EXTRA_liblanIpBasic_a_SOURCES  = padProto.c lanIpBasic.c
EXTRA_liblanIpBasic_a_SOURCES += drvLan9118.c drvLan9118IpBasic.c padStream.c bpmsim.c
EXTRA_liblanIpBasic_a_SOURCES += drvMveIpBasic.c
EXTRA_liblanIpBasic_a_SOURCES += drvAmdEthIpBasic.c

lanIpObjs_uC5282               = drvLan9118.$(OBJEXT) drvLan9118IpBasic.$(OBJEXT)
lanIpObjs_uC5282              += padProto.$(OBJEXT) padStream.$(OBJEXT) bpmsim.$(OBJEXT)

lanIpObjs_beatnik              = drvMveIpBasic.$(OBJEXT) padProto.$(OBJEXT)

lanIpObjs_pci                  = drvAmdEthIpBasic.$(OBJEXT)

lanIpObjs_uC5282_DEFS          = -DDRVLAN9118
lanIpObjs_beatnik_DEFS         = -DDRVMVE
lanIpObjs_pci_DEFS             = -DDRVAMD

AM_CPPFLAGS                   += $(lanIpObjs_$(LANIPBASIC_SUPPORT)_DEFS)

include_HEADERS                = drvLan9118.h lanIpBasic.h lanIpProto.h padProto.h udpComm.h
include_HEADERS               += padStream.h lanIpBasicSetup.h drvUdpSock.h


lib_LIBRARIES                  = liblanIpBasic.a

liblanIpBasic_a_LIBADD         = $(lanIpObjs_$(LANIPBASIC_SUPPORT))
liblanIpBasic_a_DEPENDENCIES   = $(liblanIpBasic_a_LIBADD)

bin_PROGRAMS                   = lanIpBasic
lanIpBasic_SOURCES             =
lanIpBasic_LDADD               = $(liblanIpBasic_a_OBJECTS) $(liblanIpBasic_a_LIBADD)
lanIpBasic_DEPENDENCIES        = $(lanIpBasic_LDADD)
lanIpBasic_LINK                = $(OBJLINK)

include $(top_srcdir)/rtems.am

# build a test program on a
# host computer using the ordinary network stack
HOSTPROG=padProtoHost
EXTRA_PROGRAMS=$(HOSTPROG)
exechostbin_PROGRAMS           = @HOSTPROGRAM@

# empty sources so that no automake doesn't try to use a default
padProtoHost_SOURCES           =
EXTRA_padProtoHost_SOURCES     = padProtoHost.c udpCommBSD.c padProtoNative.c
EXTRA_padProtoHost_SOURCES    += hostStream.c bpmsimNative.c

EXTRA_DIST                     = hostStream.h bpmsim.h

%Native.c: %.c
	ln -s $^ $@

padProtoHost_OBJCTS           = $(EXTRA_padProtoHost_SOURCES:%.c=%.$(OBJEXT)) 

padProtoHost_LDADD             = $(padProtoHost_OBJCTS) -lm
padProtoHost_DEPENDENCIES      = $(padProtoHost_OBJCTS)

$(padProtoHost_OBJCTS):%.$(OBJEXT):%.c
	$(HOSTCC) -c -O2 -DBSDSOCKET -I$(srcdir) -o $@ $<
