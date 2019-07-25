# Copyright 2016 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_TYPE := usertest

MODULE_SRCS += \
    $(LOCAL_DIR)/main.c \
    $(LOCAL_DIR)/region-alloc.cpp \
    $(LOCAL_DIR)/region-alloc-c-api.c

MODULE_NAME := region-alloc-test

MODULE_STATIC_LIBS := \
    system/ulib/region-alloc \
    system/ulib/mxcpp \
    system/ulib/mxtl \

MODULE_LIBS := \
    system/ulib/c \
    system/ulib/mxio \
    system/ulib/unittest

include make/module.mk
