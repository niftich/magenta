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

#include <stdint.h>

typedef struct intel_me_iotxn_data {
    uint16_t input_len;
    uint8_t _reserved;
} intel_me_iotxn_data_t;

typedef enum intel_me_remote_address {
    INTEL_ME_REMOTE_ADDR_BUS = 0x00,
    INTEL_ME_REMOTE_ADDR_CORE = 0x01,
    INTEL_ME_REMOTE_ADDR_AMT = 0x02,
    INTEL_ME_REMOTE_ADDR_ISH = 0x03,
    INTEL_ME_REMOTE_ADDR_WDT = 0x04,
    INTEL_ME_REMOTE_ADDR_MKHI = 0x07,
    INTEL_ME_REMOTE_ADDR_ICC = 0x08,
    INTEL_ME_REMOTE_ADDR_HW_ASSET = 0x0b,
    INTEL_ME_REMOTE_ADDR_PRECISE_TOUCH = 0x0c,
} intel_me_remote_address_t;
