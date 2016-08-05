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

#include <ddk/binding.h>
#include <ddk/device.h>
#include <ddk/driver.h>

#include "intel-me.h"

typedef struct intel_me_remote_device {
    mx_device_t device;
    intel_me_device_t *bus;

    intel_me_remote_address_t me_addr;
    mx_device_prop_t props[2];
} intel_me_remote_device_t;

mx_status_t intel_me_remote_device_create(intel_me_device_t* bus,
                                          intel_me_remote_address_t addr,
                                          intel_me_remote_device_t** remote);
