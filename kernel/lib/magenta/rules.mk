# Copyright 2016 The Fuchsia Authors
# Copyright (c) 2008-2015 Travis Geiselbrecht
#
# Use of this source code is governed by a MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS := \
    $(LOCAL_DIR)/channel_dispatcher.cpp \
    $(LOCAL_DIR)/diagnostics.cpp \
    $(LOCAL_DIR)/dispatcher.cpp \
    $(LOCAL_DIR)/event_dispatcher.cpp \
    $(LOCAL_DIR)/event_pair_dispatcher.cpp \
    $(LOCAL_DIR)/exception.cpp \
    $(LOCAL_DIR)/excp_port.cpp \
    $(LOCAL_DIR)/fifo_dispatcher.cpp \
    $(LOCAL_DIR)/futex_context.cpp \
    $(LOCAL_DIR)/futex_node.cpp \
    $(LOCAL_DIR)/guest_dispatcher.cpp \
    $(LOCAL_DIR)/handle.cpp \
    $(LOCAL_DIR)/handle_reaper.cpp \
    $(LOCAL_DIR)/hypervisor_dispatcher.cpp \
    $(LOCAL_DIR)/interrupt_event_dispatcher.cpp \
    $(LOCAL_DIR)/io_mapping_dispatcher.cpp \
    $(LOCAL_DIR)/job_dispatcher.cpp \
    $(LOCAL_DIR)/log_dispatcher.cpp \
    $(LOCAL_DIR)/magenta.cpp \
    $(LOCAL_DIR)/message_packet.cpp \
    $(LOCAL_DIR)/pci_device_dispatcher.cpp \
    $(LOCAL_DIR)/pci_interrupt_dispatcher.cpp \
    $(LOCAL_DIR)/pci_io_mapping_dispatcher.cpp \
    $(LOCAL_DIR)/policy_manager.cpp \
    $(LOCAL_DIR)/port_client.cpp \
    $(LOCAL_DIR)/port_dispatcher.cpp \
    $(LOCAL_DIR)/port_dispatcher_v2.cpp \
    $(LOCAL_DIR)/process_dispatcher.cpp \
    $(LOCAL_DIR)/resource_dispatcher.cpp \
    $(LOCAL_DIR)/semaphore.cpp \
    $(LOCAL_DIR)/socket_dispatcher.cpp \
    $(LOCAL_DIR)/state_tracker.cpp \
    $(LOCAL_DIR)/thread_dispatcher.cpp \
    $(LOCAL_DIR)/user_copy.cpp \
    $(LOCAL_DIR)/user_thread.cpp \
    $(LOCAL_DIR)/vm_address_region_dispatcher.cpp \
    $(LOCAL_DIR)/vm_object_dispatcher.cpp \
    $(LOCAL_DIR)/wait_set_dispatcher.cpp \
    $(LOCAL_DIR)/wait_state_observer.cpp \

MODULE_DEPS := \
    kernel/lib/dpc \
    kernel/lib/mxtl \
    kernel/dev/interrupt \
    kernel/dev/udisplay \

include make/module.mk
