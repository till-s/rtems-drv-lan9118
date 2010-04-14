#  $Id$
AUTOMAKE_OPTIONS=foreign

include $(top_srcdir)/rtems-pre.am

EXEEXT=$(OBJEXEEXT)

lanIp_SOURCES                    = lanIpBasicTest.c drvUdpSock.c hwtmr.h
lanIp_SOURCES                   += hash/lhtbl.c hash/lhtbl.h

AM_CPPFLAGS                     += -I$(srcdir)/hash

EXTRA_liblanIpBasic_ln9_a_SOURCES  = lanIpBasic.c drvLan9118.h
liblanIpBasic_ln9_a_SOURCES     = drvLan9118.c drvLan9118IpBasic.c
liblanIpBasic_ln9_a_SOURCES    += $(lanIp_SOURCES)
liblanIpBasic_mve_a_SOURCES     = drvGnrIpBasic.c gnreth_lldrv.h mve_lldrv.c
liblanIpBasic_mve_a_SOURCES    += $(lanIp_SOURCES)
EXTRA_liblanIpBasic_e1k_a_SOURCES  = lanIpBasic.c
liblanIpBasic_e1k_a_SOURCES     = drvGnrIpBasic.c gnreth_lldrv.h e1k_lldrv.c
liblanIpBasic_e1k_a_SOURCES    += $(lanIp_SOURCES)
EXTRA_liblanIpBasic_mve_a_SOURCES  = lanIpBasic.c
liblanIpBasic_pcn_a_SOURCES     = drvAmdEthIpBasic.c
liblanIpBasic_pcn_a_SOURCES    += $(lanIp_SOURCES)
EXTRA_liblanIpBasic_pcn_a_SOURCES  = lanIpBasic.c
liblanIpBasic_tsec_a_SOURCES    = drvGnrIpBasic.c gnreth_lldrv.h tsec_lldrv.c
liblanIpBasic_tsec_a_SOURCES   += $(lanIp_SOURCES)
EXTRA_liblanIpBasic_tsec_a_SOURCES = lanIpBasic.c

liblanIpBasic_e1k_a_CPPFLAGS    = $(AM_CPPFLAGS) -I$(srcdir)/../bsd_eth_drivers/if_em/

include_HEADERS_ln9             = drvLan9118.h

include_HEADERS                 = $(foreach h,$(LANIPBASIC_SUPPORT),$(include_HEADERS_$(h)))
include_HEADERS                += lanIpBasic.h lanIpProto.h
include_HEADERS                += lanIpBasicSetup.h drvUdpSock.h

lib_LIBRARIES                   = $(LANIPBASIC_SUPPORT:%=liblanIpBasic_%.a)
EXTRA_LIBRARIES                 = liblanIpBasic_ln9.a
EXTRA_LIBRARIES                += liblanIpBasic_mve.a
EXTRA_LIBRARIES                += liblanIpBasic_tsec.a
EXTRA_LIBRARIES                += liblanIpBasic_e1k.a
EXTRA_LIBRARIES                += liblanIpBasic_pcn.a

bin_PROGRAMS                    = @LANIPBASIC_PROG@
EXTRA_PROGRAMS                  = lanIpBasic_ln9
EXTRA_PROGRAMS                 += lanIpBasic_mve
EXTRA_PROGRAMS                 += lanIpBasic_tsec
EXTRA_PROGRAMS                 += lanIpBasic_e1k
EXTRA_PROGRAMS                 += lanIpBasic_pcn

lanIpBasic_ln9_SOURCES          =
lanIpBasic_mve_SOURCES          =
lanIpBasic_tsec_SOURCES         =
lanIpBasic_pcn_SOURCES          =
lanIpBasic_e1k_SOURCES          =

lanIpBasic_ln9_LDADD            = $(liblanIpBasic_ln9_a_OBJECTS)
lanIpBasic_mve_LDADD            = $(liblanIpBasic_mve_a_OBJECTS)
lanIpBasic_tsec_LDADD           = $(liblanIpBasic_tsec_a_OBJECTS)
lanIpBasic_pcn_LDADD            = $(liblanIpBasic_pcn_a_OBJECTS)
lanIpBasic_e1k_LDADD            = $(liblanIpBasic_e1k_a_OBJECTS)

lanIpBasic_ln9_DEPENDENCIES     = $(liblanIpBasic_ln9_a_OBJECTS)
lanIpBasic_mve_DEPENDENCIES     = $(liblanIpBasic_mve_a_OBJECTS)
lanIpBasic_tsec_DEPENDENCIES    = $(liblanIpBasic_tsec_a_OBJECTS)
lanIpBasic_pcn_DEPENDENCIES     = $(liblanIpBasic_pcn_a_OBJECTS)
lanIpBasic_e1k_DEPENDENCIES     = $(liblanIpBasic_e1k_a_OBJECTS)

LINK                            = $(OBJLINK)

include $(top_srcdir)/rtems.am
