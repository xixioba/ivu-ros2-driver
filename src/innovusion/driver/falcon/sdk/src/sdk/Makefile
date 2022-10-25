include version_gen.mk

OBJCOPY_OPT = -g -R .note -R .comment
OBJCOPY_OPT_DYNA = -g -R .note -R .comment --strip-all --keep-symbols api.export

STATIC_LIB = libinnolidarsdk.a
ifeq ($(ARCH_TAG), -mingw64)
DYNA_LIB = libinnolidarsdk.dll
else
DYNA_LIB = libinnolidarsdk.so
endif

SRC_DIR = ../../src/sdk
OBJ_DIR = ../../obj/sdk
DEP_DIR = ../../dep/sdk
OUT_DIR = ../../lib

CPPLINT = ../../build/cpplint.py
CC ?= gcc
CXX ?= g++
CFLAGS ?= -I .. -D_GLIBCXX_USE_CXX11_ABI=0 -O2 -g2 -std=gnu++11 -Wall -Werror -fpermissive \
                        $(CUSTOMER_RELEASE_DEF) $(CUSTOMER_SOURCE_DEF)
LINKFLAGS ?= -Wl,-Bstatic -static
LINKFLAGS_DYNA ?=
AR ?= ar
OBJCOPY ?= objcopy
INC ?= -I./
OTHER_CFLAGS ?= -fPIC
CFLAGS += $(INC) $(OTHER_CFLAGS)
OTHER_LIB_BUILD_CFLAG ?=

ifeq ($(ARCH_TAG), -mingw64)
	INNO_LIB_BUILD_CFLAG = -L ../../lib -linnolidarsdkcommon -linnolidarutils -linnolidarsdkclient
endif
ifeq ($(ARCH_TAG), -macos)
	INNO_LIB_BUILD_CFLAG = ../../lib/libinnolidarsdkcommon.a ../../lib/libinnolidarutils.a ../../lib/libinnolidarsdkclient.a -lc++
	CFLAGS += -Wno-error=unused-but-set-variable -Wno-error=infinite-recursion -Wno-error=inconsistent-missing-override -Wno-error=format -Wno-error=empty-body
	OBJCOPY_OPT =
	OBJCOPY_OPT_DYNA =
endif

ifeq ($(ARCH_TAG), -arm)
	MARCH_VALUE ?= armv8-a+crc
	MTUNE_VALUE ?= cortex-a53
	CFLAGS += -march=$(MARCH_VALUE) -mtune=$(MTUNE_VALUE)
endif

ifeq ($(ARCH_TAG), -mingw64)
	SRCS := $(wildcard $(SRC_DIR)/*.cpp $(SRC_DIR)/*/*.cpp)
else
	SRCS := $(wildcard $(SRC_DIR)/*.cpp)
endif
OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))
DEPS = $(patsubst $(SRC_DIR)/%.cpp, $(DEP_DIR)/%.d, $(SRCS))

ifneq ($(wildcard $(SRC_DIR)/*stage_noise_filter.cpp),)
.PHONY: build
build: lint $(OUT_DIR)/$(STATIC_LIB) $(OUT_DIR)/$(DYNA_LIB)  $(OUT_DIR)/libinnolidarstage_noise_filter.a
else
OBJS += $(OBJ_DIR)/stage_noise_filter.o
.PHONY: build
build: lint $(OUT_DIR)/$(STATIC_LIB) $(OUT_DIR)/$(DYNA_LIB)
endif

$(OBJ_DIR) :
	mkdir -p $(OBJ_DIR)
ifeq ($(ARCH_TAG), -mingw64)
	mkdir -p $(OBJ_DIR)/mman-win32
endif

$(OUT_DIR) :
	mkdir -p $(OUT_DIR)

$(DEP_DIR) :
	mkdir -p $(DEP_DIR)
ifeq ($(ARCH_TAG), -mingw64)
	mkdir -p $(DEP_DIR)/mman-win32
endif

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

$(OUT_DIR)/$(DYNA_LIB):: $(OUT_DIR)
$(OUT_DIR)/$(DYNA_LIB):: $(OBJS)
ifeq ($(ARCH_TAG), -mingw64)
	$(CC) -shared -Wl,-soname,$(DYNA_LIB) -o $@ $^ $(INNO_LIB_BUILD_CFLAG) $(OTHER_LIB_BUILD_CFLAG)
	$(OBJCOPY) $(OBJCOPY_OPT) $(OBJCOPY_OPT_DYNA) $@ $@
else
ifeq ($(ARCH_TAG), -macos)
	$(CC) -shared -Wl,-install_name,$(DYNA_LIB).$(DYNA_LIB_MAJ) -o $@- $^ $(INNO_LIB_BUILD_CFLAG) $(OTHER_LIB_BUILD_CFLAG)
else
	$(CC) -shared -Wl,-soname,$(DYNA_LIB).$(DYNA_LIB_MAJ) -o $@- $^ $(INNO_LIB_BUILD_CFLAG) $(OTHER_LIB_BUILD_CFLAG)
endif
	$(OBJCOPY) $(OBJCOPY_OPT) $(OBJCOPY_OPT_DYNA) $@- $@.$(DYNA_LIB_MAJ).$(DYNA_LIB_MIN).$(DYNA_LIB_BUILD)
	rm -f $@-
	rm -f $(OUT_DIR)/$(DYNA_LIB).$(DYNA_LIB_MAJ) $(OUT_DIR)/$(DYNA_LIB)
	ln -s $(DYNA_LIB).$(DYNA_LIB_MAJ).$(DYNA_LIB_MIN).$(DYNA_LIB_BUILD) $(OUT_DIR)/$(DYNA_LIB).$(DYNA_LIB_MAJ)
	ln -s $(DYNA_LIB).$(DYNA_LIB_MAJ) $(OUT_DIR)/$(DYNA_LIB)
	-cd -
endif

$(OUT_DIR)/libinnolidarstage_noise_filter.a: $(OBJ_DIR)/stage_noise_filter.o
ifeq ($(ARCH_TAG), -macos)
	$(AR) -r $@ $^
else
	$(AR) -r -o $@ $^
endif

$(OBJ_DIR)/stage_noise_filter.o:
ifneq ($(wildcard $(SRC_DIR)/*stage_noise_filter.cpp),)
	$(CC) -c $(CFLAGS) stage_noise_filter.cpp -o $@
	$(CC) -MM $(CFLAGS) -MT"$@" stage_noise_filter.cpp > $(DEP_DIR)/stage_noise_filter.d
else
	$(AR) -x -v $(OUT_DIR)/libinnolidarstage_noise_filter.a stage_noise_filter.o
	mv stage_noise_filter.o $(OBJ_DIR)
endif

$(OBJ_DIR)/%.o: %.cpp | $(OBJ_DIR) $(DEP_DIR)
	$(CC) -c $(CFLAGS) $*.cpp -o $(OBJ_DIR)/$*.o
	$(CC) -MM $(CFLAGS) -MT"$@" $*.cpp > $(DEP_DIR)/$*.d

.PHONY: lint
lint: lint_checked

lint_checked: $(wildcard *.h) $(wildcard *.cpp)
	$(CPPLINT) --root=.. $?
	touch lint_checked

clean:
	rm -fr lint_checked $(OBJ_DIR) $(DEP_DIR) $(OUT_DIR)/$(STATIC_LIB) $(OUT_DIR)/$(DYNA_LIB)*
