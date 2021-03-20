#
# Makefile
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
# library file name
APP_NAME = ModelLoader

##################################################################################
# top directory name
TOP_DIR_NAME = $(APP_NAME)

##################################################################################
# compiler
# OS dependency
ifeq ($(OS),Linux)
	CXX:=g++ -std=c++11
else ifeq ($(OS),QNX)
  CXX:=QCC -Vgcc_ntox86_cpp
else
  $(error unknown OS $(OS))
endif

##################################################################################
# local directory
LIB_DIR = ./lib
BIN_DIR = ./bin
SRC_DIR = ./src
SRC_APP_DIR = $(SRC_DIR)/$(APP_NAME)
SRC_HRPUTIL_DIR = $(SRC_DIR)/hrpUtil
SRC_HRPCOLLISION_DIR = $(SRC_DIR)/hrpCollision
SRC_HRPCOLLISION_SUBDIR = $(SRC_HRPCOLLISION_DIR)/Opcode/Ice
SRC_HRPCOLLISION_SUBDIR += $(SRC_HRPCOLLISION_DIR)/Opcode
SRC_HRPMODEL_DIR = $(SRC_DIR)/hrpModel
SRC_HRP_DIR =   $(SRC_HRPUTIL_DIR)
SRC_HRP_DIR +=  $(SRC_HRPCOLLISION_DIR)
SRC_HRP_DIR +=  $(SRC_HRPCOLLISION_SUBDIR)
SRC_HRP_DIR +=  $(SRC_HRPMODEL_DIR)

##################################################################################
# test directory
TEST_SRC_DIR = ./test
TEST_SRC_UTIL_DIR = $(TEST_SRC_DIR)/util

##################################################################################
# include directory
INCLUDE_DIR = ./include
INCLUDE_MAIN_DIR = $(INCLUDE_DIR)/$(TOP_DIR_NAME)
INCLUDE_HRPUTIL_DIR = $(INCLUDE_DIR)/hrpUtil
INCLUDE_HRPUTIL_SUBDIR =  $(INCLUDE_HRPUTIL_DIR)/Jpeg-6b
INCLUDE_HRPUTIL_SUBDIR += $(INCLUDE_HRPUTIL_DIR)/lpng1232
INCLUDE_HRPUTIL_SUBDIR += $(INCLUDE_HRPUTIL_DIR)/zlib123
INCLUDE_HRPCOLLISION_DIR = $(INCLUDE_DIR)/hrpCollision
INCLUDE_HRPMODEL_DIR = $(INCLUDE_DIR)/hrpModel
INCLUDES = $(INCLUDE_DIR)
INCLUDES += $(INCLUDE_MAIN_DIR)
INCLUDES += $(INCLUDE_HRPUTIL_DIR)
INCLUDES += $(INCLUDE_HRPUTIL_SUBDIR)
INCLUDES += $(INCLUDE_HRPCOLLISION_DIR)
INCLUDES += $(INCLUDE_HRPCOLLISION_DIR)/Opcode
INCLUDES += $(INCLUDE_HRPMODEL_DIR)
#
# OS dependency
ifeq ($(OS),Linux)
	INCLUDE_EIGEN_DIR = /usr/include/eigen3
	INCLUDES += $(INCLUDE_EIGEN_DIR)
else ifeq ($(OS),QNX)
	INCLUDES += /usr/pkg/include
endif
INCLUDES += /usr/local/include

#
# INCLUDES_PATH(add prefix -I)
INCLUDES_PATH = $(addprefix -I, $(INCLUDES))

##################################################################################
# library directoxry
LINK_DIRS =  -L. -L$(LIB_DIR) -L/usr/lib
ifeq ($(OS),QNX)
	LINK_DIRS += -L/usr/pkg/lib
endif
LINK_DIRS += -L/usr/local/lib

#
# link (pay attension to linking-order)
LINK_GTEST = -lgtest_main -lgtest
LINK_JPEG = $(LIB_DIR)/libjpeg.a
LINK_PNG =  $(LIB_DIR)/libpng.a
LINK_LAPACK = -latlas -llapack -lf2c -lblas -lcblas
# boost
LINK_BOOST_COMPONETS = filesystem signals system regex program_options
LINK_BOOST = $(addprefix -lboost_, $(LINK_BOOST_COMPONETS))
#
LINK = $(LINK_DIRS)
LINK += -lm
LINK += $(LINK_GTEST)
#
LINK += $(LINK_JPEG)
LINK += $(LINK_PNG)
LINK += $(LINK_LAPACK)
LINK += $(LINK_BOOST)
#
# OS dependency
ifeq ($(OS),Linux)
	LINK += -lpthread -ldl
else ifeq ($(OS),QNX)
	LINK += -lz
endif
#
#
##################################################################################
# option
CFLAGS = -g3 -Wall -D$(UNAME) -D_REENTRANT
#
# OS dependency
ifeq ($(OS),QNX)
	CFLAGS += -DEIGEN_MPL2_ONLY
endif

##################################################################################
# library & application
SLIB_APP = $(LIB_DIR)/lib$(APP_NAME).a
# LIB_APP = $(LIB_DIR)/lib$(APP_NAME).so
EXE_APP  = $(BIN_DIR)/$(APP_NAME)
TEST_APP = $(BIN_DIR)/unit_test
#
ALL_SRC =  $(wildcard $(addsuffix /*.cpp, $(SRC_DIR)))
ALL_SRC += $(wildcard $(addsuffix /*.cpp, $(SRC_APP_DIR)))
EXE_SRC :=$(SRC_DIR)/main.cpp
LIB_SRC = $(filter-out $(EXE_SRC), $(ALL_SRC))
LIB_SRC += $(wildcard $(addsuffix /*.cpp, $(SRC_HRP_DIR)))
TEST_SRC:=$(wildcard $(TEST_SRC_DIR)/*.cpp) $(wildcard $(TEST_SRC_UTIL_DIR)/*.cpp)
#
LIB_OBJS=$(LIB_SRC:%.cpp=%.o)
EXE_OBJS =$(EXE_SRC:%.cpp=%.o)
TEST_OBJS=$(TEST_SRC:%.cpp=%.o)

##################################################################################
# Target
COMPILE_TARGETS = $(SLIB_APP) $(LIB_APP) $(EXE_APP) $(TEST_APP)
all: compile_title $(COMPILE_TARGETS)
compile_title:
	@echo
	@echo "COMPILE_TARGETS="$(COMPILE_TARGETS)"\n"
	@echo ---- $(MAKE) $(COMPILE_TARGETS) "("$(shell basename $(shell pwd))")" ----- "\n"


# separate compile -- make staic library
$(SLIB_APP): $(LIB_OBJS) $(LINK_JPEG) $(LINK_PNG)
	@echo "\n  "$^" --> "$@"\n"
	@if [ ! -d $(LIB_DIR) ]; then \
		mkdir -p $(LIB_DIR); \
	fi
	ar rcs $@ $^
# @rm $(LIB_OBJS)
# separate compile -- make shared library
$(LIB_APP): $(LIB_OBJS) $(LINK_JPEG) $(LINK_PNG)
	@echo "\n  "$^" --> "$@"\n"
	@if [ ! -d $(LIB_DIR) ]; then \
		mkdir -p $(LIB_DIR); \
	fi
	$(eval LIB_CFLAGS = $(CFLAGS) -fPIC)
	$(CXX) -shared -o $@ $^ $(LIB_CFLAGS) $(LINK)
# @rm $(LIB_OBJS)

# separated compile -- make executing application
$(EXE_APP): $(EXE_OBJS) $(SLIB_APP)
	@echo "\n  "$^" --> "$@"\n"
	@if [ ! -d $(BIN_DIR) ]; then \
		mkdir -p $(BIN_DIR); \
	fi
	$(CXX) -o $@ $^ $(CFLAGS) $(LINK)
# @rm $(EXE_OBJS)

# separated compile -- make unit_test application
$(TEST_APP): $(TEST_OBJS) $(SLIB_APP)
	@echo "\n  "$^" --> "$@"\n"
	@if [ ! -d $(BIN_DIR) ]; then \
		mkdir -p $(BIN_DIR); \
	fi
	$(CXX) -o $@ $^ $(CFLAGS) $(LINK)
# @rm $(TEST_OBJS)

### common compile -- make object file
%.o: %.cpp
	@echo "\n  "$<" --> "$@"\n"
	@if [ "$(LIB_APP)" != "" ]; then \
		echo "$(CXX) -c $(CFLAGS) -fPIC $(INCLUDES_PATH) -o $@ $<"; \
		$(CXX) -c $(CFLAGS) -fPIC $(INCLUDES_PATH) -o $@ $<;\
	else \
		echo "$(CXX) -c $(CFLAGS) $(INCLUDES_PATH) -o $@ $<"; \
		$(CXX) -c $(CFLAGS) $(INCLUDES_PATH) -o $@ $<; \
	fi

### other depended 3rd-library
$(LINK_JPEG):
	make -f Jpeg-6b.mk

$(LINK_PNG):
	make -f lpng1232.mk

# make clean
clean:
	rm -f $(LIB_OBJS)
	rm -f $(EXE_OBJS)
	rm -f $(TEST_OBJS)
	rm -f *~ core
	rm -f $(INCLUDE_DIR)/*~
	rm -f $(SRC_DIR)/*~
	rm -f $(SRC_APP_DIR)/*~
	rm -f $(SRC_HRPUTIL_DIR)/*~
	rm -f $(SRC_HRPCOLLISION_DIR)/*~
	rm -f $(SRC_HRPMODEL_DIR)/*~
	make -f Jpeg-6b.mk clean
	make -f lpng1232.mk clean

# make cleanall
cleanall: clean
	rm -f $(SLIB_APP)
	rm -f $(EXE_APP)
	rm -f $(TEST_APP)
	make -f Jpeg-6b.mk cleanall
	make -f lpng1232.mk cleanall
