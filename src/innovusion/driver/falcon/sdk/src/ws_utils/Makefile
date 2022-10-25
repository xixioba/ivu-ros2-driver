include version_gen.mk

STATIC_LIB = libinnolidarwsutils.a
ifeq ($(ARCH_TAG), -mingw64)
DYNA_LIB = libinnolidarwsutils.dll
else
DYNA_LIB = libinnolidarwsutils.so
endif

SRC_DIR = ../../src/ws_utils
OBJ_DIR = ../../obj/ws_utils
DEP_DIR = ../../dep/ws_utils
OUT_DIR = ../../lib

CPPLINT = ../../build/cpplint.py
CC ?= gcc
CXX ?= g++
CFLAGS ?= -I .. -I ../.. -D_GLIBCXX_USE_CXX11_ABI=0 -O2 -g2 -std=gnu++11 -Wall -Werror -fpermissive \
                        $(CUSTOMER_RELEASE_DEF) $(CUSTOMER_SOURCE_DEF)
LINKFLAGS ?= -pthread -Wl,-Bstatic -static
LINKFLAGS_DYNA ?= -pthread
AR ?= ar
OBJCOPY ?= objcopy
INC ?= -I./ $(BOOST_INC)
OTHER_CFLAGS ?= -fPIC
CFLAGS += $(INC) $(OTHER_CFLAGS)

ifeq ($(ARCH_TAG), -arm)
	CFLAGS += -march=armv8-a+crc -mtune=cortex-a53 -DARCH_ARM64
endif
ifeq ($(ARCH_TAG), -macos)
	CFLAGS += $(OPENSSL_INC) -DOPENSSL_API_COMPAT=0x10000000L -Wno-error=deprecated-declarations -Wno-deprecated-declarations -Wno-error=format -Wno-error=dangling-gsl
	INNO_LIB_BUILD_CFLAG = ../../lib/libinnolidarutils.a $(OPENSSL_LIB) -lc++
endif
ifeq ($(ARCH_TAG), -mingw64)
	CFLAGS += $(OPENSSL_INC)
	INNO_LIB_BUILD_CFLAG = -L../../lib -linnolidarutils $(OPENSSL_LIB) -lwsock32 -lws2_32
endif

SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))
DEPS = $(patsubst $(SRC_DIR)/%.cpp, $(DEP_DIR)/%.d, $(SRCS))

.PHONY: build
build: lint $(OUT_DIR)/$(STATIC_LIB) $(OUT_DIR)/$(DYNA_LIB)

$(OBJ_DIR) :
	mkdir -p $(OBJ_DIR)

$(OUT_DIR) :
	mkdir -p $(OUT_DIR)

$(DEP_DIR) :
	mkdir -p $(DEP_DIR)

# pull in dependency info for *existing* .o files
-include $(DEPS)

$(OUT_DIR)/$(STATIC_LIB):: $(OUT_DIR)
$(OUT_DIR)/$(STATIC_LIB):: $(OBJS)
ifeq ($(ARCH_TAG), -macos)
	$(AR) -r $@- $^
else
	$(AR) -r -o $@- $^
endif
	$(OBJCOPY) $(OBJCOPY_OPT) $@- $@
	rm -f $@-

$(OUT_DIR)/$(DYNA_LIB)::  $(OUT_DIR)
$(OUT_DIR)/$(DYNA_LIB):: $(OBJS)
ifeq ($(ARCH_TAG), -mingw64)
	$(CC) -shared -Wl,-soname,$(DYNA_LIB) -o $@ $^ $(INNO_LIB_BUILD_CFLAG)
else
ifeq ($(ARCH_TAG), -macos)
	$(CC) -shared -Wl,-install_name,$(DYNA_LIB).$(DYNA_LIB_MAJ) -o $@- $^ $(INNO_LIB_BUILD_CFLAG)
else
	$(CC) -shared -Wl,-soname,$(DYNA_LIB).$(DYNA_LIB_MAJ) -o $@- $^ $(INNO_LIB_BUILD_CFLAG)
endif
	$(OBJCOPY) $(OBJCOPY_OPT) $(OBJCOPY_OPT_DYNA) $@- $@.$(DYNA_LIB_MAJ).$(DYNA_LIB_MIN).$(DYNA_LIB_BUILD)
	rm -f $@-
	rm -f $(OUT_DIR)/$(DYNA_LIB).$(DYNA_LIB_MAJ) $(OUT_DIR)/$(DYNA_LIB)
	ln -s $(DYNA_LIB).$(DYNA_LIB_MAJ).$(DYNA_LIB_MIN).$(DYNA_LIB_BUILD) $(OUT_DIR)/$(DYNA_LIB).$(DYNA_LIB_MAJ)
	ln -s $(DYNA_LIB).$(DYNA_LIB_MAJ) $(OUT_DIR)/$(DYNA_LIB)
	-cd -
endif

$(OBJ_DIR)/%.o:: %.cpp | $(OBJ_DIR) $(DEP_DIR)
	$(CC) -c $(CFLAGS) $*.cpp -o $(OBJ_DIR)/$*.o
	$(CC) -MM $(CFLAGS) -MT"$@" $*.cpp > $(DEP_DIR)/$*.d

.PHONY: lint
lint: lint_checked

lint_checked: $(wildcard *.h) $(wildcard *.cpp) $(wildcard server_ws/server_ws.hpp)
	$(CPPLINT) --linelength=100 --root=.. $?
	touch lint_checked

clean:
	rm -fr lint_checked $(OBJ_DIR) $(DEP_DIR) $(OUT_DIR)/$(STATIC_LIB) $(OUT_DIR)/$(DYNA_LIB)*
