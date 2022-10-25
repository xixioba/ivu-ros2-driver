###
#  Copyright (C) 2021 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##
SRC_DIR = ../../../src/utils/unitest
OBJ_DIR = ../../../obj/utils/unitest
DEP_DIR = ../../../dep/utils/unitest
LIB_DIR = ../../../lib
BIN_DIR = ../../../unitest/bin/
TARGETS =
TARGETS_STATIC = inno_pc_unitestserver

CPPLINT = ../../../build/cpplint.py
CC ?= gcc
CXX ?= g++
STRIP ?= strip
CFLAGS = -D_GLIBCXX_USE_CXX11_ABI=0 -O2 -g2 -Wall -Werror -fpermissive -std=gnu++11 $(INC_DIR)
LINKFLAGS = -Wl,--whole-archive -lpthread -Wl,--no-whole-archive -Wl,-Bstatic -static
DYNA_LINKFLAGS = -pthread
INC_DIR = -I../ -I../../ -I../../../src/ -I../../../thirdparty/ $(BOOST_INC)
INNO_LIBS =  -linnolidargtest -linnolidarutils
OTHER_LIBS = $(BOOST_LIB) -lboost_system -lssl -lcrypto -ldl -lstdc++ -lm

SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))
DEPS := $(patsubst $(SRC_DIR)/%.cpp, $(DEP_DIR)/%.d, $(SRCS))

INC ?= -I./
OTHER_CFLAGS ?= -fPIC
UNITESTFLAGS ?=
OTHER_CFLAGS +=  -I ../../../thirdparty/gtest/gtest/googletest/include
CFLAGS += $(INC) $(OTHER_CFLAGS) $(UNITESTFLAGS)

ifeq ($(ARCH_TAG), -arm)
	CFLAGS += -march=armv8-a+crc -mtune=cortex-a53 -DARCH_ARM64
endif

.PHONY: build
build: lint $(TARGETS_STATIC)

.PHONY: all
all: build

$(OBJ_DIR) :
	mkdir -p $(OBJ_DIR)

$(OUT_DIR) :
	mkdir -p $(OUT_DIR)

$(DEP_DIR) :
	mkdir -p $(DEP_DIR)

$(BIN_DIR) :
	mkdir -p $(BIN_DIR)

-include $(DEPS)

$(OBJ_DIR)/%.o: %.cpp | $(OBJ_DIR) $(DEP_DIR)
	$(CC) -c $(CFLAGS) $*.cpp -o $(OBJ_DIR)/$*.o
	$(CC) -MM $(CFLAGS) -MT"$@" $*.cpp > $(DEP_DIR)/$*.d

inno_pc_unitestserver: $(OBJS) | $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -L $(LIB_DIR) $(INNO_LIBS) $(LINKFLAGS) $(OTHER_LIBS)
	cp -f $@ $@_debug
	$(STRIP) $@
	mv $@   $(BIN_DIR)
	mv $@_debug   $(BIN_DIR)

.PHONY: clean
clean:
	rm -rf $(OBJ_DIR) $(DEP_DIR) $(TARGETS) $(TARGETS_STATIC)* lint_checked
	rm -rf $(BIN_DIR)

.PHONY: lint
lint: lint_checked

lint_checked: $(wildcard *.h) $(wildcard *.cpp)
	$(CPPLINT) --root=.. $?
	touch lint_checked
