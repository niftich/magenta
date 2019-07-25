# Copyright 2016 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_NAME := netsvc

MODULE_TYPE := userapp

MODULE_SRCS += \
    $(LOCAL_DIR)/netsvc.c \
    $(LOCAL_DIR)/netfile.c \
    $(LOCAL_DIR)/device_id.c

MODULE_STATIC_LIBS := system/ulib/inet6

MODULE_LIBS := system/ulib/mxio system/ulib/launchpad system/ulib/magenta system/ulib/c

include make/module.mk
