#
# Makefile for libjpeg.a
#
#
# Copyright (C) 2019, kdaic
# All right reserved.
# This software is made under the MIT License.
# http://opensource.org/licenses/mit-license.php
#
##################################################################################
OS:=$(shell uname -s)
UNAME := $(shell uname)

##################################################################################
# compiler
# OS dependency
ifeq ($(OS),Linux)
	CXX:=gcc
else ifeq ($(OS),QNX)
  CXX:=QCC
else
  $(error unknown OS $(OS))
endif

##################################################################################
# local directory
LIB_DIR = ./lib
SRC_DIR = ./src/hrpUtil/Jpeg-6b
INCLUDE_DIR = ./include/hrpUtil/Jpeg-6b
CFLAGS = -O2 -I$(INCLUDE_DIR)

SOURCES = jcapimin.c jcapistd.c jccoefct.c jccolor.c jcdctmgr.c jchuff.c \
        jcinit.c jcmainct.c jcmarker.c jcmaster.c jcomapi.c jcparam.c \
        jcphuff.c jcprepct.c jcsample.c jctrans.c jdapimin.c jdapistd.c \
        jdatadst.c jdatasrc.c jdcoefct.c jdcolor.c jddctmgr.c jdhuff.c \
        jdinput.c jdmainct.c jdmarker.c jdmaster.c jdmerge.c jdphuff.c \
        jdpostct.c jdsample.c jdtrans.c jerror.c jfdctflt.c jfdctfst.c \
        jfdctint.c jidctflt.c jidctfst.c jidctint.c jidctred.c jquant1.c \
        jquant2.c jutils.c jmemmgr.c jmemnobs.c

##################################################################################
#
LIB_SRC = $(addprefix $(SRC_DIR)/, $(SOURCES))

LIB_OBJS=$(LIB_SRC:%.c=%.o)

SLIB_APP = $(LIB_DIR)/libjpeg.a

##################################################################################
# Target
all: compile_title $(SLIB_APP)

compile_title:
	@echo
	@echo "COMPILE_TARGETS="$(SLIB_APP)"\n"
	@echo ---- $(MAKE) $(SLIB_APP) "("$(shell basename $(shell pwd))")" ----- "\n"

# separate compile -- make staic library
$(SLIB_APP): $(LIB_OBJS)
	@echo "\n  "$^" --> "$@"\n"
	@if [ ! -d $(LIB_DIR) ]; then \
		mkdir -p $(LIB_DIR); \
	fi
	ar rcs $@ $^
# @rm $(LIB_OBJS)

### common compile -- make object file
%.o: %.c
	@echo "\n  "$<" --> "$@"\n"
	@echo "$(CXX) -c $(CFLAGS) $(INCLUDES_PATH) -o $@ $<";
	$(CXX) -c $(CFLAGS) -o $@ $<;

# make clean
clean:
	rm -f $(SRC_DIR)/*.o

# make cleanall
cleanall: clean
	rm -f $(SLIB_APP)
