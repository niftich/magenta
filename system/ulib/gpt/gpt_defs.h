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

#define GPT_MAGIC (0x5452415020494645ull) // 'EFI PART'
#define GPT_HEADER_SIZE 0x5c
#define GPT_ENTRY_SIZE  0x80
#define GPT_GUID_STRLEN 37
#define GPT_NAME_LEN 72

typedef struct gpt_header {
    uint64_t magic;
    uint32_t revision;
    uint32_t size;
    uint32_t crc32;
    uint32_t reserved0;
    uint64_t current;
    uint64_t backup;
    uint64_t first;
    uint64_t last;
    uint8_t guid[16];
    uint64_t entries;
    uint32_t entries_count;
    uint32_t entries_size;
    uint32_t entries_crc;
    uint8_t reserved[420]; // for 512-byte block size
} gpt_header_t;

typedef struct gpt_entry {
    uint8_t type[16];
    uint8_t guid[16];
    uint64_t first;
    uint64_t last;
    uint64_t flags;
    uint8_t name[72];
} gpt_entry_t;
