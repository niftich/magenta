# Copyright 2017 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_TYPE := userlib

MODULE_SRCS += $(LOCAL_DIR)/tftp.c

MODULE_EXPORT := a

#MODULE_SO_NAME := tftp
MODULE_LIBS := system/ulib/magenta system/ulib/c

include make/module.mk

MODULE := $(LOCAL_DIR).test

MODULE_TYPE := usertest

MODULE_SRCS := $(LOCAL_DIR)/tftp-test.cpp

MODULE_NAME := tftp-test

MODULE_STATIC_LIBS := system/ulib/tftp system/ulib/mxtl system/ulib/mxcpp

MODULE_LIBS := system/ulib/unittest system/ulib/mxio system/ulib/c

include make/module.mk

MODULE := $(LOCAL_DIR).tftp-example

MODULE_TYPE := hostapp

MODULE_SRCS := $(LOCAL_DIR)/tftp.c $(LOCAL_DIR)/tftp-example.c

MODULE_NAME := tftp-example

MODULE_COMPILEFLAGS += -I$(LOCAL_DIR)/include -std=c11

include make/module.mk
