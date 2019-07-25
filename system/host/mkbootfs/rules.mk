# Copyright 2017 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

LOCAL_DIR := $(GET_LOCAL_DIR)

LZ4_DIR := third_party/ulib/lz4

MODULE := $(LOCAL_DIR)

MODULE_TYPE := hostapp

MODULE_SRCS += \
    $(LZ4_DIR)/lz4.c \
    $(LZ4_DIR)/lz4frame.c \
    $(LZ4_DIR)/lz4hc.c \
    $(LZ4_DIR)/xxhash.c \
    $(LOCAL_DIR)/mkbootfs.c \

MODULE_CFLAGS := -I$(LZ4_DIR)/include/lz4

include make/module.mk
