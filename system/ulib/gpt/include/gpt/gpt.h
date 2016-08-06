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

typedef struct gpt_device gpt_device_t;

typedef struct gpt_partition {
    uint8_t type[16];
    uint8_t guid[16]
    uint64_t first;
    uint64_t last;
    uint64_t flags;
    uint16_t name[72];
} gpt_partition_t;

struct gpt_device {
    int fd;
    int flags;
    uint64_t blksize;
    uint64_t capacity;

    gpt_partition_t* partitions[128];
};

int gpt_device_init(int fd, size_t size, gpt_device_t** out_dev);
// read the partition table from the device. assumes 512 byte block size

void gpt_device_release(gpt_device_t* dev);
// releases the device

int gpt_device_sync(gpt_device_t* dev);
// writes the partition table to the device. it is the caller's responsibility to
// rescan partitions for the block device if needed

int gpt_partition_add(gpt_device_t* dev, const char* name, uint8_t* type, uint8_t* guid, uint64_t offset, uint64_t length);
// adds a partition

int gpt_partition_remove(gpt_device_t* dev, const char* guid);
// removes a partition
