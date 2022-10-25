###
#  Copyright (C) 2021 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##

STATIC_LIB = libinnolidarsdk.a libinnolidarsdkclient.a libinnolidarsdkcommon.a libinnolidarutils.a libinnolidarwsutils.a
ifeq ($(ARCH_TAG), -mingw64)
DYNA_LIB = libinnolidarsdk.dll libinnolidarsdkclient.dll libinnolidarsdkcommon.dll libinnolidarutils.dll libinnolidarwsutils.dll
else
DYNA_LIB = libinnolidarsdk.so libinnolidarsdkclient.so libinnolidarsdkcommon.so libinnolidarutils.so libinnolidarwsutils.so
endif
SRC_DIR = ../../apps/pcs
OBJ_DIR = ../../obj/pcs
DEP_DIR = ../../dep/pcs
LIB_DIR = ../../lib

ifdef MINGW64
TARGETS_STATIC = inno_pc_server.exe inno_pc_client.exe
TARGETS = inno_pc_server_dynamic.exe inno_pc_client_dynamic.exe
else
TARGETS = inno_pc_server_dynamic inno_pc_client_dynamic
TARGETS_STATIC = inno_pc_server inno_pc_client
endif

CPPLINT = ../../build/cpplint.py
CC ?= gcc
CXX ?= g++
STRIP ?= strip
CFLAGS_ALL = -D_GLIBCXX_USE_CXX11_ABI=0 -O2 -g2 -Wall -fpermissive -std=gnu++11 $(INC_DIR)
LINKFLAGS = -Wl,--whole-archive -lpthread -Wl,--no-whole-archive -Wl,-Bstatic -static
DYNA_LINKFLAGS = -pthread
INC_DIR = -I../ -I../../ -I../../src/ $(BOOST_INC)
INNO_LIBS = -linnolidarsdk -linnolidarsdkclient -linnolidarsdkcommon -linnolidarutils -linnolidarwsutils
INNO_CLIENT_LIBS = -linnolidarsdkclient -linnolidarsdkcommon -linnolidarutils -linnolidarwsutils
OTHER_LIBS = $(BOOST_LIB) -lssl -lcrypto -ldl -lstdc++ -lm
ifeq ($(ROSRELEASE), kinetic)
	OTHER_LIBS += -lboost_system
endif
UBUNTU_VER := $(shell lsb_release -rs)
ifeq ($(UBUNTU_VER), 20.04)
    CFLAGS = $(CFLAGS_ALL)
else
    CFLAGS = $(CFLAGS_ALL) -Werror
endif
$(info INC_DIR: $(INC_DIR))
$(info OTHER_LIBS: $(OTHER_LIBS))
$(info CFLAGS: $(CFLAGS))


UBUNTU_VER := $(shell lsb_release -rs)
ifeq ($(UBUNTU_VER), 20.04)
    CFLAGS = $(CFLAGS_ALL)
else
    CFLAGS = $(CFLAGS_ALL) -Werror
endif


SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))
DEPS := $(patsubst $(SRC_DIR)/%.cpp, $(DEP_DIR)/%.d, $(SRCS))
STATIC_LIB_FILES := $(patsubst %, $(LIB_DIR)/%, $(STATIC_LIB))
DYNA_LIB_FILES := $(patsubst %, $(LIB_DIR)/%, $(DYNA_LIB))

INC ?= -I./
OTHER_CFLAGS ?= -fPIC
CFLAGS += $(INC) $(OTHER_CFLAGS)

ifeq ($(ARCH_TAG), -arm)
	CFLAGS += -march=armv8-a+crc -mtune=cortex-a53 -DARCH_ARM64
endif
ifeq ($(ARCH_TAG), -macos)
	CFLAGS += $(GPERF_INC) $(BOOST_INC) $(OPENSSL_INC) -DOPENSSL_API_COMPAT=0x10000000L -Wno-error=deprecated-declarations -Wno-error=unused-private-field -Wno-error=format -Wno-error=dangling-gsl
	LINKFLAGS = -pthread
	INNO_LIBS = ../../lib/libinnolidarsdk.a ../../lib/libinnolidarsdkclient.a ../../lib/libinnolidarsdkcommon.a ../../lib/libinnolidarutils.a ../../lib/libinnolidarwsutils.a
	INNO_CLIENT_LIBS = ../../lib/libinnolidarsdkclient.a ../../lib/libinnolidarsdkcommon.a ../../lib/libinnolidarutils.a ../../lib/libinnolidarwsutils.a
	OTHER_LIBS = $(BOOST_LIB) $(OPENSSL_LIB) $(GPERF_LIB) -lc++
endif
ifeq ($(ARCH_TAG), -mingw64)
	CFLAGS += $(OPENSSL_INC)
	INNO_LIBS = -Wl,--allow-multiple-definition ../../lib/libinnolidarsdk.a ../../lib/libinnolidarsdkclient.a ../../lib/libinnolidarwsutils.a ../../lib/libinnolidarsdkcommon.a ../../lib/libinnolidarutils.a
	INNO_CLIENT_LIBS = -Wl,--allow-multiple-definition ../../lib/libinnolidarsdkclient.a ../../lib/libinnolidarwsutils.a ../../lib/libinnolidarsdkcommon.a ../../lib/libinnolidarutils.a
	OTHER_LIBS = $(BOOST_LIB) $(OPENSSL_LIB) -lstdc++ -lm -lwsock32 -lws2_32 -lwinmm
endif

.PHONY: build
build: lint $(TARGETS_STATIC)

.PHONY: dynamic
dynamic: lint $(TARGETS)

.PHONY: all
all: build dynamic

$(OBJ_DIR) :
	mkdir -p $(OBJ_DIR)

$(OUT_DIR) :
	mkdir -p $(OUT_DIR)

$(DEP_DIR) :
	mkdir -p $(DEP_DIR)

-include $(DEPS)

$(OBJ_DIR)/%.o: %.cpp | $(OBJ_DIR) $(DEP_DIR)
	$(CC) -c $(CFLAGS) $*.cpp -o $(OBJ_DIR)/$*.o
	$(CC) -MM $(CFLAGS) -MT"$@" $*.cpp > $(DEP_DIR)/$*.d

inno_pc_server_dynamic: $(OBJS) $(DYNA_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $(OBJS)  -L $(LIB_DIR) $(patsubst %.a,%.so,$(INNO_LIBS)) $(DYNA_LINKFLAGS) $(OTHER_LIBS)
	$(STRIP) $@
ifeq ($(ARCH_TAG), -macos)
	install_name_tool -change libinnolidarsdkclient.so.0 @rpath/libinnolidarsdkclient.so $@
	install_name_tool -change libinnolidarsdkcommon.so.0 @rpath/libinnolidarsdkcommon.so $@
	install_name_tool -change libinnolidarsdk.so.0 @rpath/libinnolidarsdk.so $@
	install_name_tool -change libinnolidarutils.so.0 @rpath/libinnolidarutils.so $@
	install_name_tool -change libinnolidarwsutils.so.0 @rpath/libinnolidarwsutils.so $@
	install_name_tool -add_rpath "@loader_path/../../lib" $@
endif

inno_pc_server: $(OBJS) $(STATIC_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -L $(LIB_DIR) $(INNO_LIBS) $(LINKFLAGS) $(OTHER_LIBS)
	cp -f $@ $@_debug
	$(STRIP) $@

inno_pc_server.exe: $(OBJS) $(STATIC_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -L $(LIB_DIR) $(INNO_LIBS) $(LINKFLAGS) $(OTHER_LIBS)
	cp -f $@ inno_pc_server_debug.exe
	$(STRIP) $@

inno_pc_server_dynamic.exe: $(OBJS) $(DYNA_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $(OBJS)  -L $(LIB_DIR) $(patsubst %.a,%.dll,$(INNO_LIBS)) $(DYNA_LINKFLAGS) $(OTHER_LIBS)
	$(STRIP) $@

inno_pc_client_dynamic: $(OBJS) $(DYNA_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -L $(LIB_DIR) $(patsubst %.a,%.so,$(INNO_CLIENT_LIBS)) $(DYNA_LINKFLAGS) $(OTHER_LIBS)
	$(STRIP) $@
ifeq ($(ARCH_TAG), -macos)
	install_name_tool -change libinnolidarsdkclient.so.0 @rpath/libinnolidarsdkclient.so $@
	install_name_tool -change libinnolidarsdkcommon.so.0 @rpath/libinnolidarsdkcommon.so $@
	install_name_tool -change libinnolidarsdk.so.0 @rpath/libinnolidarsdk.so $@
	install_name_tool -change libinnolidarutils.so.0 @rpath/libinnolidarutils.so $@
	install_name_tool -change libinnolidarwsutils.so.0 @rpath/libinnolidarwsutils.so $@
	install_name_tool -add_rpath "@loader_path/../../lib" $@
endif

inno_pc_client: $(OBJS) $(STATIC_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -L $(LIB_DIR) $(INNO_CLIENT_LIBS) $(LINKFLAGS) $(OTHER_LIBS)
	cp -f $@ $@_debug
	$(STRIP) $@

inno_pc_client.exe: $(OBJS) $(STATIC_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -L $(LIB_DIR) $(INNO_CLIENT_LIBS) $(LINKFLAGS) $(OTHER_LIBS)
	cp -f $@ inno_pc_client_debug.exe
	$(STRIP) $@

inno_pc_client_dynamic.exe: $(OBJS) $(DYNA_LIB_FILES)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -L $(LIB_DIR) $(patsubst %.a,%.dll,$(INNO_CLIENT_LIBS)) $(DYNA_LINKFLAGS) $(OTHER_LIBS)
	$(STRIP) $@

.SILENT: $(LIB_DIR)/libinnolidarsdk.a
$(LIB_DIR)/libinnolidarsdk.a:
	echo "libinnolidarsdk.a does not exist."
	echo "If you are running 'make' in an SDK distrubution, please run 'make dynamic'."
	echo "Exiting..."
	false

.PHONY: clean
clean:
	rm -rf $(OBJ_DIR) $(DEP_DIR) $(TARGETS) $(TARGETS_STATIC) inno_pc_server_debug inno_pc_server_debug.exe inno_pc_client_debug inno_pc_client_debug.exe lint_checked

.PHONY: clean_dynamic
clean_dynamic:
	rm -rf $(OBJ_DIR) $(DEP_DIR) $(TARGETS) lint_checked

.PHONY: lint
lint: lint_checked

lint_checked: $(wildcard *.h) $(wildcard *.cpp)
	$(CPPLINT) --root=.. $?
	touch lint_checked
