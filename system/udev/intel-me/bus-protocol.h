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
#include <ddk/protocol/pci.h>

#include "intel-me.h"
#include "remote.h"

void intel_me_reset(intel_me_device_t* device);
void intel_me_set_irq_enable(intel_me_device_t* device, bool enable);
void intel_me_clear_irq(intel_me_device_t* device);

bool intel_me_ready_for_send(intel_me_device_t* device);
uint8_t intel_me_pending_data(intel_me_device_t* device);

mx_status_t intel_me_send_message(
        intel_me_device_t* device,
        uint8_t host_addr);
void intel_me_recv_message(intel_me_device_t* device);

void intel_me_test(
        intel_me_device_t* device, intel_me_remote_device_t* remote);
