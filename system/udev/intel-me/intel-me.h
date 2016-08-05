// Copyright 2016 The Fuchsia Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/protocol/intel-me.h>

#include <runtime/completion.h>
#include <runtime/mutex.h>
#include <runtime/thread.h>

#define MAX_RUNNING_TXNS 1

typedef struct intel_me_device {
    mx_device_t device;

    // device registers
    struct intel_me_registers* regs;
    uint64_t regs_size;
    mx_handle_t regs_handle;

    // irq processing resources
    mx_handle_t irq_handle;
    mxr_thread_t* irq_thread;

    // txn processing resources
    mxr_completion_t worker_completion;
    mxr_thread_t* worker_thread;
    list_node_t txn_list;
    mxr_mutex_t txn_list_lock;
    iotxn_t* running_txns[MAX_RUNNING_TXNS];

    // lock for ensuring complete messages are written to the bus
    mxr_mutex_t bus_lock;
} intel_me_device_t;

// This struct is the actual form of intel_me_iotxn_data_t
typedef struct {
    uint16_t input_len;
    uint8_t remote_addr;
} real_iotxn_data_t;

_Static_assert(sizeof(real_iotxn_data_t) == sizeof(intel_me_iotxn_data_t),
               "real struct and shadow struct have different sizes");

struct intel_me_remote_device;

void intel_me_iotxn_queue(
        intel_me_device_t* device,
        struct intel_me_remote_device* remote,
        iotxn_t* txn);

void intel_me_complete_txn(
        intel_me_device_t* device, int slot_num,
        mx_status_t status, mx_off_t written);
