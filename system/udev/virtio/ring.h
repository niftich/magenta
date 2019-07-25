// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#pragma once

#include <magenta/types.h>
#include <stddef.h>

#include "trace.h"
#include "virtio_ring.h"

namespace virtio {

class Device;

class Ring {
public:
    Ring(Device* device);
    ~Ring();

    mx_status_t Init(uint16_t index, uint16_t count);

    void FreeDesc(uint16_t desc_index);
    void FreeDescChain(uint16_t chain_head);
    uint16_t AllocDesc();
    struct vring_desc* AllocDescChain(uint16_t count, uint16_t* start_index);
    void SubmitChain(uint16_t desc_index);
    void Kick();

    struct vring_desc* DescFromIndex(uint16_t index) {
        return &ring_.desc[index];
    }

    template <typename T>
    void IrqRingUpdate(T free_chain);

private:
    Device* device_ = nullptr;

    mx_paddr_t ring_pa_ = 0;
    uintptr_t ring_va_ = 0;
    size_t ring_va_len_ = 0;

    uint16_t index_ = 0;

    vring ring_ = {};
};

// perform the main loop of finding free descriptor chains and passing it to a passed in function
template <typename T>
inline void Ring::IrqRingUpdate(T free_chain) {
    //TRACEF("used flags 0x%hhx idx 0x%hhx last_used %u\n",
    //        ring_.used->flags, ring_.used->idx, ring_.last_used);

    // find a new free chain of descriptors
    unsigned int cur_idx = ring_.used->idx;
    for (unsigned int i = ring_.last_used; i != (cur_idx & ring_.num_mask); i = (i + 1) & ring_.num_mask) {
        //TRACEF("looking at idx %u\n", i);

        struct vring_used_elem* used_elem = &ring_.used->ring[i];
        //TRACEF("used chain id %u, len %u\n", used_elem->id, used_elem->len);

        // free the chain
        free_chain(used_elem);

        ring_.last_used = ((ring_.last_used + 1) & ring_.num_mask) & 0xffff;
    }
}

void virtio_dump_desc(const struct vring_desc* desc);

} // namespace virtio
