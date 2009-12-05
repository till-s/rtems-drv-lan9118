#  $Id$
AUTOMAKE_OPTIONS=foreign

include $(top_srcdir)/rtems-pre.am

EXEEXT=$(OBJEXEEXT)

liblanIpBasic_a_SOURCES        = lanIpBasicTest.c drvUdpSock.c hwtmr.h
liblanIpBasic_a_SOURCES       += lhtbl.c lhtbl.h

EXTRA_liblanIpBasic_a_SOURCES  = padProto.c lanIpBasic.c
EXTRA_liblanIpBasic_a_SOURCES += drvLan9118.c drvLan9118IpBasic.c padStream.c bpmsim.c
EXTRA_liblanIpBasic_a_SOURCES += drvGnrIpBasic.c
EXTRA_liblanIpBasic_a_SOURCES += drvAmdEthIpBasic.c
EXTRA_liblanIpBasic_a_SOURCES += gnreth_lldrv.h
EXTRA_liblanIpBasic_a_SOURCES += mve_lldrv.c
EXTRA_liblanIpBasic_a_SOURCES += e1k_lldrv.c

lanIpObjs_lan9118              = drvLan9118.$(OBJEXT) drvLan9118IpBasic.$(OBJEXT)
lanIpObjs_pad                  = padProto.$(OBJEXT) padStream.$(OBJEXT) bpmsim.$(OBJEXT)

lanIpObjs_gnreth               = drvGnrIpBasic.$(OBJEXT) padProto.$(OBJEXT) mve_lldrv.$(OBJEXT)

lanIpObjs_mve                  = mve_lldrv.$(OBJEXT)
lanIpObjs_pcn                  = drvAmdEthIpBasic.$(OBJEXT)
lanIpObjs_e1k                  = e1k_lldrv.$(OBJEXT)

#lanIpObjs_pci                  = drvGnrIpBasic.$(OBJEXT) e1k_lldrv.$(OBJEXT)
lanIpObjs_e1k_CPPFLAGS         = -I$(srcdir)/../bsd_eth_drivers/if_em/

AM_CPPFLAGS                   += $(lanIpObjs_$(LANIPBASIC_SUPPORT)_CPPFLAGS)

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
