###
#  Copyright (C) 2021 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##
.PHONY: all
all: build

.PHONY: build
ifndef MINGW64
build: utils ws_utils sdk_common sdk_client sdk
else
build: lnk utils ws_utils sdk_common sdk_client sdk
endif

.PHONY: build_client
ifndef MINGW64
build_client: utils ws_utils sdk_common sdk_client
else
build_client: lnk utils ws_utils sdk_common sdk_client
endif

.PHONY: utils
utils:
	@$(MAKE) -C $@

.PHONY: ws_utils
ws_utils:
	@$(MAKE) -C $@

.PHONY: sdk_common
sdk_common: | utils
	@$(MAKE) -C $@

.PHONY: sdk_client
sdk_client: | sdk_common
	@$(MAKE) -C $@

.PHONY: sdk
sdk: | sdk_client
	@$(MAKE) -C $@

.PHONY: lnk
lnk:
ifneq ($(wildcard */*version_gen.mk.lnk),)
	-$(shell rm */*version_gen.mk.lnk)
	-$(shell cp -f sdk_common/version_gen.mk sdk/version_gen.mk)
	-$(shell cp -f sdk_common/version_gen.mk sdk_client/version_gen.mk)
	-$(shell cp -f sdk_common/version_gen.mk utils/version_gen.mk)
	-$(shell cp -f sdk_common/version_gen.mk ws_utils/version_gen.mk)
endif

.PHONY: clean
clean: lnk
	@$(MAKE) -C utils clean
	@$(MAKE) -C ws_utils clean
	@$(MAKE) -C sdk_common clean
	@$(MAKE) -C sdk_client clean
	-@$(MAKE) -C sdk clean

.PHONY: test
test:
	@$(MAKE) -C utils test

