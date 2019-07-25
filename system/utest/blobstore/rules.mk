# Copyright 2017 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_TYPE := usertest

MODULE_NAME := blobstore-test

MODULE_SRCS := \
    $(LOCAL_DIR)/blobstore.cpp

MODULE_STATIC_LIBS := \
    system/ulib/merkle \
    third_party/ulib/cryptolib \
    system/ulib/mxalloc \
    system/ulib/mxcpp \
    system/ulib/mxtl \

MODULE_LIBS := \
    system/ulib/mxio \
    system/ulib/c \
    system/ulib/fs-management \
    system/ulib/magenta \
    system/ulib/unittest \

include make/module.mk
