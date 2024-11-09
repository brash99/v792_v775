#
# Description:  Makefile for c792Lib.o
#   This driver is specific to VxWorks BSPs and must be compiled
#   with access to vxWorks headers.
#
# SVN: $Rev: 402 $
#
# Uncomment the line before for compiling for a Linux/Intel-based Controler
ARCH=Linux
#

#Check Operating system we are using
ifndef OSNAME
  OSNAME := $(subst -,_,$(shell uname))
endif

ifndef ARCH
  ARCH = VXWORKSPPC
endif

ifeq ($(ARCH),VXWORKSPPC)
INCDIR=/site/vxworks/5.5/ppc/target/h
CC = ccppc
LD = ldppc
DEFS = -mcpu=604 -DCPU=PPC604 -DVXWORKS -D_GNU_TOOL -DVXWORKSPPC
INCS = -fno-for-scope -fno-builtin -fvolatile -fstrength-reduce -mlongcall -I. -I$(INCDIR)
CFLAGS = -O $(DEFS)
endif

ifeq ($(ARCH),VXWORKS68K51)
INCDIR=/site/vxworks/5.3/68k/target/h
CC = cc68k
DEFS = -DCPU=MC68040 -DVXWORKS -DVXWORKS68K51
INCS = -Wall -mc68020 -fvolatile -fstrength-reduce -nostdinc -I. -I$(INCDIR)
CFLAGS = -O $(DEFS)
endif

ifeq ($(ARCH),Linux)

ifndef LINUXVME_LIB
	LINUXVME_LIB	= ${CODA}/extensions/linuxvme/lib
endif
ifndef LINUXVME_INC
	LINUXVME_INC	= ${CODA}/extensions/linuxvme/include
endif
CC = gcc
AR = ar
RANLIB = ranlib
DEFS = -DJLAB
INCS = -I. -I${LINUXVME_INC}
CFLAGS = -O ${DEFS} -O2  -L. -L${LINUXVME_LIB}
ifdef DEBUG
CFLAGS += -Wall -g
endif

endif

ifeq ($(ARCH),Linux)
all: echoarch libc792.a libc775.a
else
all: echoarch c792Lib.o c775Lib.o
endif

c792Lib.o: caen792Lib.c c792Lib.h
	$(CC) -c $(CFLAGS) $(INCS) -o $@ caen792Lib.c

c775Lib.o: caen775Lib.c c775Lib.h
	$(CC) -c $(CFLAGS) $(INCS) -o $@ caen775Lib.c


libc792.a: c792Lib.o
	$(CC) -fpic -shared $(CFLAGS) $(INCS) -o libc792.so caen792Lib.c
	$(AR) ruv libc792.a c792Lib.o
	$(RANLIB) libc792.a

links: libc792.a
	ln -sf $(PWD)/libc792.a $(LINUXVME_LIB)/libc792.a
	ln -sf $(PWD)/libc792.so $(LINUXVME_LIB)/libc792.so
	ln -sf $(PWD)/c792Lib.h $(LINUXVME_INC)/c792Lib.h

libc775.a: c775Lib.o
	$(CC) -fpic -shared $(CFLAGS) $(INCS) -o libc775.so caen775Lib.c
	$(AR) ruv libc775.a c775Lib.o
	$(RANLIB) libc775.a

links2: libc775.a
	ln -sf $(PWD)/libc775.a $(LINUXVME_LIB)/libc775.a
	ln -sf $(PWD)/libc775.so $(LINUXVME_LIB)/libc775.so
	ln -sf $(PWD)/c775Lib.h $(LINUXVME_INC)/c775Lib.h

clean:
	rm -f *.o *.so *.a

echoarch:
	echo "Make for $(ARCH)"

rol:
	make -f Makefile-rol

rolclean:
	make -f Makefile-rol clean
