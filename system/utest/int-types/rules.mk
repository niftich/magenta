# Copyright 2016 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_TYPE := usertest

MODULE_SRCS := $(LOCAL_DIR)/int-types.cpp

MODULE_NAME := int-types-test

MODULE_STATIC_LIBS := system/ulib/mxtl

MODULE_LIBS := system/ulib/unittest system/ulib/mxio system/ulib/c

include make/module.mk
