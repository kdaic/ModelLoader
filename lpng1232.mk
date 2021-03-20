#
# Makefile for libpng.a
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
SRC_DIR = ./src/hrpUtil/lpng1232
INCLUDE_DIR = ./include/hrpUtil/lpng1232 ./include/hrpUtil/zlib
CFLAGS = -O2 $(addprefix -I, $(INCLUDE_DIR))

SOURCES = png.c pngerror.c pngget.c pngmem.c pngpread.c pngread.c \
        pngrio.c pngrtran.c pngrutil.c pngset.c pngtrans.c pngwio.c \
        pngwrite.c pngwtran.c pngwutil.c

##################################################################################
#
LIB_SRC = $(addprefix $(SRC_DIR)/, $(SOURCES))

LIB_OBJS=$(LIB_SRC:%.c=%.o)

SLIB_APP = $(LIB_DIR)/libpng.a

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
