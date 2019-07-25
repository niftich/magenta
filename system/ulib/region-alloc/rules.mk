# Copyright 2016 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_TYPE := userlib

MODULE_SRCS += \
    $(LOCAL_DIR)/region-alloc.cpp \
    $(LOCAL_DIR)/region-alloc-c-api.cpp

MODULE_STATIC_LIBS := \
    system/ulib/mxtl \
    system/ulib/mxcpp

include make/module.mk
