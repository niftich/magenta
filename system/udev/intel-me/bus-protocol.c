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

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <magenta/syscalls-ddk.h>

#include "bus-protocol.h"

struct intel_me_registers {
    // Below, the following abbrieviations are used:
    // cb: circular buffer
    // cs: control status
    volatile uint32_t host_cb_write_window;
    volatile uint32_t host_cs;
    volatile uint32_t me_cb_read_window;
    volatile uint32_t me_cs_host_access;
    volatile uint32_t host_power_gating_cs;
} __PACKED;

// host_cs/me_cs_host_access field extraction macros
#define CS_CB_DEPTH(cs) ((cs) >> 24)
#define CS_CB_WP(cs) (((cs) >> 16) & 0xff)
#define CS_CB_RP(cs) (((cs) >> 8) & 0xff)
#define CS_RESET(cs) !!((cs) & (1<<4))
#define CS_READY(cs) !!((cs) & (1<<3))
#define CS_INTERRUPT_GEN(cs) !!((cs) & (1<<2))
#define CS_INTERRUPT_STATUS(cs) !!((cs) & (1<<1))
#define CS_INTERRUPT_ENABLE(cs) !!((cs) & (1<<0))

// host_cs control bits
#define HOST_CS_RESET_BIT (1<<4)
#define HOST_CS_READY_BIT (1<<3)
#define HOST_CS_INTERRUPT_GEN_BIT (1<<2)
#define HOST_CS_INTERRUPT_CLEAR_BIT (1<<1)
#define HOST_CS_INTERRUPT_ENABLE_BIT (1<<0)

#define MAX_MESSAGE_LEN ((1<<9)-1)

// We offset by 2, since host addrs 0 and 1 are reserved
#define SLOT_NUM_TO_HOST_ADDR(n) ((n) + 5)
#define HOST_ADDR_TO_SLOT_NUM(n) ((n) - 5)

#define MESSAGE_HDR_MESSAGE_COMPLETE (1<<15)

union message_hdr {
    struct {
        uint8_t me_addr;
        uint8_t host_addr;
        // only low 9 bits of len can be used to specify len
        // MSB is used as the Message Complete bit
        uint16_t len;
    } fields;
    uint32_t raw;
} __PACKED;

void intel_me_reset(intel_me_device_t* device) {
    // If device is already in reset, clear the bit so we can start the sequence
    // again.
    if (CS_RESET(device->regs->host_cs)) {
        device->regs->host_cs &= ~HOST_CS_RESET_BIT;
    }

    device->regs->host_cs |= HOST_CS_RESET_BIT | HOST_CS_INTERRUPT_GEN_BIT | HOST_CS_INTERRUPT_CLEAR_BIT;

    // TODO(teisenbe): Add a timeout
    // Wait for the ME coprocessor to be ready
    while (!CS_READY(device->regs->me_cs_host_access));

    // Clear the reset condition
    uint32_t host_cs = device->regs->host_cs;
    host_cs &= ~HOST_CS_RESET_BIT;
    device->regs->host_cs = host_cs;

    // Announce host is ready and enable interrupts
    host_cs = device->regs->host_cs;
    host_cs |= HOST_CS_READY_BIT | HOST_CS_INTERRUPT_GEN_BIT | HOST_CS_INTERRUPT_ENABLE_BIT;
    device->regs->host_cs = host_cs;
}

bool intel_me_ready_for_send(intel_me_device_t* device) {
    // We don't need to hold the lock here, since only one thread
    // can send data (the iotxn worker).  The irq thread invokes this too,
    // but only as an optimistic check.

    if (!CS_READY(device->regs->me_cs_host_access)) {
        printf("MEI NOT READY FOR SEND\n");
        return false;
    }

    uint32_t status = device->regs->host_cs;
    uint8_t depth = CS_CB_DEPTH(status);
    uint8_t write_ptr = CS_CB_WP(status);
    uint8_t read_ptr = CS_CB_RP(status);

    // read_ptr is always behind or at the write_ptr
    uint8_t bytes_in_use;
    if (write_ptr >= read_ptr) {
        bytes_in_use = write_ptr - read_ptr;
    } else {
        // Wrapped around
        bytes_in_use = write_ptr + depth - read_ptr;
    }

    uint8_t avail_space = depth - bytes_in_use;
    // We check greater than 1, to leave space for a sentinel gap that allows us
    // to differentiate between full and empty
    printf("TX SPACE AVAIL: %d\n", avail_space - 1);
    return avail_space > 1;
}

uint8_t intel_me_pending_data(intel_me_device_t* device) {
    // We don't need to hold the lock here, since the only thread that
    // can read data (the irq thread) is the only one that invokes this
    // function.
    if (!CS_READY(device->regs->me_cs_host_access)) {
        return false;
    }

    uint32_t status = device->regs->me_cs_host_access;
    uint8_t depth = CS_CB_DEPTH(status);
    uint8_t write_ptr = CS_CB_WP(status);
    uint8_t read_ptr = CS_CB_RP(status);

    // read_ptr is always behind or at the write_ptr
    uint8_t bytes_in_use;
    if (write_ptr >= read_ptr) {
        bytes_in_use = write_ptr - read_ptr;
    } else {
        // Wrapped around
        bytes_in_use = write_ptr + depth - read_ptr;
    }

    return bytes_in_use;
}

void intel_me_set_irq_enable(intel_me_device_t* device, bool enable) {
    mxr_mutex_lock(&device->bus_lock);
    uint32_t val = device->regs->host_cs & ~HOST_CS_INTERRUPT_CLEAR_BIT;
    if (enable) {
        val |= HOST_CS_INTERRUPT_ENABLE_BIT;
    } else {
        val &= ~HOST_CS_INTERRUPT_ENABLE_BIT;
    }
    device->regs->host_cs = val;
    mxr_mutex_unlock(&device->bus_lock);
}

void intel_me_clear_irq(intel_me_device_t* device) {
    mxr_mutex_lock(&device->bus_lock);
    device->regs->host_cs |= HOST_CS_INTERRUPT_CLEAR_BIT;
    mxr_mutex_unlock(&device->bus_lock);
}

mx_status_t intel_me_send_message(
        intel_me_device_t* device,
        uint8_t slot_num) {

    iotxn_t* txn = device->running_txns[slot_num];
    assert(txn);
    real_iotxn_data_t* pdata = iotxn_pdata(txn, real_iotxn_data_t);

    printf("SENDING MESSAGE, TXN LEN: %lld\n", txn->length);
    assert(txn->length == MAX_MESSAGE_LEN);
    uint16_t len = pdata->input_len;

    if (len > MAX_MESSAGE_LEN) {
        return ERR_INVALID_ARGS;
    }

    printf("SENDING MESSAGE to %d from %d, len %d\n", pdata->remote_addr, SLOT_NUM_TO_HOST_ADDR(slot_num), len);

    union message_hdr hdr;
    hdr.fields.me_addr = pdata->remote_addr;
    hdr.fields.host_addr = SLOT_NUM_TO_HOST_ADDR(slot_num);
    hdr.fields.len = len | MESSAGE_HDR_MESSAGE_COMPLETE;

    // TODO(teisenbe): Add flow control
    // TODO(teisenbe): Check ME_RDY bit

    mxr_mutex_lock(&device->bus_lock);

    if (!intel_me_ready_for_send(device)) {
        mxr_mutex_unlock(&device->bus_lock);
        return ERR_BAD_STATE;
    }

    // Send header
    device->regs->host_cb_write_window = hdr.raw;

    // Copy request to static send_buffer protected by bus lock
    static uint8_t send_buffer[MAX_MESSAGE_LEN];
    txn->ops->copyfrom(txn, send_buffer, len, 0);

    // Send the data in groups of 4 bytes
    unsigned int remaining = len;
    while (remaining >= 4) {
        uint32_t data = *(uint32_t*)(send_buffer + len - remaining);

        device->regs->host_cb_write_window = data;
        remaining -= 4;
    }

    if (remaining == 0) {
        goto done;
    }

    // Send the trailing partial dword if it exists
    uint32_t last_dword = 0;
    memcpy(&last_dword, send_buffer + len - remaining, remaining);
    device->regs->host_cb_write_window = last_dword;

done:
    // Send the ME coprocessor an interrupt
    device->regs->host_cs |= HOST_CS_INTERRUPT_GEN_BIT;

    mxr_mutex_unlock(&device->bus_lock);
    return NO_ERROR;
}

void intel_me_recv_message(intel_me_device_t* device) {
    uint16_t remaining_message_bytes = 0;
    unsigned int offset = 0;
    union message_hdr hdr;
    uint8_t slot_num = MAX_RUNNING_TXNS;
    iotxn_t *txn = NULL;
    bool drop_message = false;

    mxr_mutex_lock(&device->bus_lock);
    while (true) {
        uint8_t remaining_data = intel_me_pending_data(device);
        if (remaining_data == 0) {
            // TODO: Handle running out of buffer space mid transaction....
            assert(!txn && !drop_message);
            break;
        }
        while (remaining_data > 0) {
            uint32_t data = device->regs->me_cb_read_window;
            remaining_data--;

            if (!txn && !drop_message) {
                // This is a header dword
                hdr.raw = data;
                printf("HEADER DWORD %08x\n", data);
                remaining_message_bytes = hdr.fields.len & MAX_MESSAGE_LEN;
                slot_num = HOST_ADDR_TO_SLOT_NUM(hdr.fields.host_addr);
                txn = device->running_txns[slot_num];
                offset = 0;
                if (txn) {
                    real_iotxn_data_t* pdata = iotxn_pdata(txn, real_iotxn_data_t);
                    assert(hdr.fields.me_addr == pdata->remote_addr);
                } else {
                    printf("WARNING: Dropping message for slot %d\n", slot_num);
                    drop_message = true;
                }

                printf("RECEIVING MESSAGE from %d to %d, len %d\n", hdr.fields.me_addr, hdr.fields.host_addr, remaining_message_bytes);
            } else {
                unsigned int to_copy = sizeof(data);
                if (remaining_message_bytes < sizeof(data)) {
                    to_copy = remaining_message_bytes;
                }

                if (!drop_message) {
                    txn->ops->copyto(txn, &data, to_copy, offset);
                }
                offset += to_copy;
                remaining_message_bytes -= to_copy;
            }

            if (remaining_message_bytes == 0) {
                remaining_message_bytes = 0;

                if (txn) {
                    txn->actual += offset;
                    if (hdr.fields.len & MESSAGE_HDR_MESSAGE_COMPLETE) {
                        intel_me_complete_txn(device, slot_num, NO_ERROR, txn->actual);
                    }
                }

                txn = NULL;
                drop_message = false;
            }
        }

        // Let the coprocessor know we've read the data
        device->regs->host_cs |= HOST_CS_INTERRUPT_GEN_BIT;
    }
    mxr_mutex_unlock(&device->bus_lock);
}

static void intel_me_test_callback(iotxn_t* txn) {
    printf("Request finished: %d\n", txn->status);
    uint32_t data[13] = { 0 };
    assert(txn->actual <= sizeof(data));

    txn->ops->copyfrom(txn, data, txn->actual, 0);

    for (unsigned int i = 0; i < 13; ++i) {
        printf("Byte read in %p: %08x\n", txn->context, data[i]);
    }
    txn->ops->release(txn);
}

void intel_me_test(
        intel_me_device_t* device, intel_me_remote_device_t* remote) {

    iotxn_t *txn = NULL;
    mx_status_t status = iotxn_alloc(&txn, 0, MAX_MESSAGE_LEN, 0);
    if (status != NO_ERROR) {
        printf("Failed to allocate iotxn: %d\n", status);
        return;
    }
    txn->length = MAX_MESSAGE_LEN;

    uint32_t data = 0x1;
    txn->ops->copyto(txn, &data, sizeof(data), 0);

    txn->complete_cb = intel_me_test_callback;
    txn->context = 0;
    txn->protocol = MX_PROTOCOL_INTEL_ME;
    intel_me_iotxn_data_t* pdata = iotxn_pdata(txn, intel_me_iotxn_data_t);
    pdata->input_len = sizeof(data);

    printf("QUEUEING TRANSACTION\n");
    intel_me_iotxn_queue(device, remote, txn);

    txn = NULL;
    status = iotxn_alloc(&txn, 0, MAX_MESSAGE_LEN, 0);
    if (status != NO_ERROR) {
        printf("Failed to allocate iotxn: %d\n", status);
        return;
    }
    txn->length = MAX_MESSAGE_LEN;

    data = 0x5;
    txn->ops->copyto(txn, &data, sizeof(data), 0);

    txn->complete_cb = intel_me_test_callback;
    txn->context = (void*)1;
    txn->protocol = MX_PROTOCOL_INTEL_ME;
    pdata = iotxn_pdata(txn, intel_me_iotxn_data_t);
    pdata->input_len = sizeof(data);

    printf("QUEUEING TRANSACTION 2\n");
    intel_me_iotxn_queue(device, remote, txn);
}
