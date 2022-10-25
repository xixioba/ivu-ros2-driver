###
#  Copyright (C) 2021 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##

.PHONY: build
build: src example pcs

.PHONY: all
all: build

.PHONY: src
src:
ifneq ($(wildcard src/sdk/*),)
	$(MAKE) -C src build
else
	$(MAKE) -C src build_client
endif

.PHONY: example
example: | src
	$(MAKE) -C apps/example

.PHONY: pcs
pcs: | src
	$(MAKE) -C apps/pcs dynamic

.PHONY: ros
ros: | src
	$(MAKE) -C apps/ros

.PHONY: clean
clean:
	$(MAKE) -C src clean
	$(MAKE) -C apps/example clean
	@! test -f apps/pcs/Makefile || $(MAKE) -C apps/pcs clean_dynamic
	@! test -f apps/ros/Makefile || $(MAKE) -C apps/ros clean
