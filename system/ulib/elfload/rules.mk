# Copyright 2016 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_TYPE := userlib

MODULE_SRCS := $(LOCAL_DIR)/elf-load.c

# This library refers to Magenta system calls and so needs libmagenta
# headers.  But the library itself has no dependencies, so users must
# ensure that the libmagenta entry points are available.
MODULE_LIBS := system/ulib/magenta

include make/module.mk
