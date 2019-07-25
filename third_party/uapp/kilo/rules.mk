# Copyright 2016 The Fuchsia Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_TYPE := userapp

MODULE_SRCS += $(LOCAL_DIR)/kilo.c

MODULE_NAME := kilo

MODULE_LIBS := system/ulib/mxio system/ulib/c

ifeq ($(call TOBOOL,$(USE_CLANG)),false)
MODULE_CFLAGS += -Wno-discarded-qualifiers
else
MODULE_CFLAGS += -Wno-incompatible-pointer-types-discards-qualifiers
endif

USER_MANIFEST_LINES += src/kilo.c=$(LOCAL_DIR)/kilo.c

include make/module.mk
