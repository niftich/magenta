// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <magenta/hw/usb.h>
#include <magenta/hw/usb-hub.h>
#include <magenta/types.h>
#include <magenta/listnode.h>
#include <sync/completion.h>
#include <limits.h>
#include <stdbool.h>
#include <threads.h>

#include <ddk/device.h>
#include <ddk/protocol/pci.h>
#include <ddk/protocol/usb-bus.h>

#include "xhci-hw.h"
#include "xhci-root-hub.h"
#include "xhci-trb.h"

// choose ring sizes to allow each ring to fit in a single page
#define COMMAND_RING_SIZE (PAGE_SIZE / sizeof(xhci_trb_t))
#define TRANSFER_RING_SIZE (PAGE_SIZE / sizeof(xhci_trb_t))
#define EVENT_RING_SIZE (PAGE_SIZE / sizeof(xhci_trb_t))
#define ERST_ARRAY_SIZE 1

#define XHCI_RH_USB_2 0 // index of USB 2.0 virtual root hub device
#define XHCI_RH_USB_3 1 // index of USB 2.0 virtual root hub device
#define XHCI_RH_COUNT 2 // number of virtual root hub devices

// state for endpoint's current transfer
typedef struct xhci_transfer_state {
    iotxn_phys_iter_t   phys_iter;
    uint32_t            packet_count;       // remaining packets to send
    uint8_t             ep_type;
    uint8_t             direction;
    bool                needs_data_event;   // true if we still need to queue data event TRB
    bool                needs_status;       // true if we still need to queue status TRB
} xhci_transfer_state_t;

typedef struct xhci_endpoint {
    xhci_endpoint_context_t* epc;
    xhci_transfer_ring_t transfer_ring;
    list_node_t queued_txns;    // iotxns waiting to be processed
    iotxn_t* current_txn;       // iotxn currently being processed
    list_node_t pending_txns;   // processed txns waiting for completion, including current_txn
    xhci_transfer_state_t* transfer_state;  // transfer state for current_txn
    mtx_t lock;
    bool enabled;
} xhci_endpoint_t;

typedef struct xhci_slot {
    // buffer for our device context
    io_buffer_t buffer;
    xhci_slot_context_t* sc;
    // epcs point into DMA memory past sc
    xhci_endpoint_t eps[XHCI_NUM_EPS];
    uint32_t hub_address;
    uint32_t port;
    uint32_t rh_port;
    usb_speed_t speed;
} xhci_slot_t;

typedef struct xhci xhci_t;

typedef void (*xhci_command_complete_cb)(void* data, uint32_t cc, xhci_trb_t* command_trb,
                                         xhci_trb_t* event_trb);

typedef struct {
    xhci_command_complete_cb callback;
    void* data;
} xhci_command_context_t;

struct xhci {
    // the device we implement
    mx_device_t* mxdev;
    mx_device_t* bus_mxdev;
    usb_bus_protocol_t* bus_protocol;

    pci_protocol_t* pci_proto;
    bool legacy_irq_mode;
    mx_handle_t irq_handle;
    mx_handle_t mmio_handle;
    mx_handle_t cfg_handle;
    thrd_t irq_thread;

    // used by the start thread
    mx_device_t* parent;

    // MMIO data structures
    xhci_cap_regs_t* cap_regs;
    xhci_op_regs_t* op_regs;
    volatile uint32_t* doorbells;
    xhci_runtime_regs_t* runtime_regs;

    // DMA data structures
    uint64_t* dcbaa;
    mx_paddr_t dcbaa_phys;

    xhci_transfer_ring_t command_ring;
    mtx_t command_ring_lock;
    xhci_command_context_t* command_contexts[COMMAND_RING_SIZE];

    // One event ring for now, but we will have multiple if we use multiple interruptors
#define INTERRUPTOR_COUNT 1
    xhci_event_ring_t event_rings[INTERRUPTOR_COUNT];
    erst_entry_t* erst_arrays[INTERRUPTOR_COUNT];
    mx_paddr_t erst_arrays_phys[INTERRUPTOR_COUNT];

    size_t page_size;
    size_t max_slots;
    size_t max_interruptors;
    size_t context_size;
    // true if controller supports large ESIT payloads
    bool large_esit;

    // total number of ports for the root hub
    uint32_t rh_num_ports;

    // state for virtual root hub devices
    // one for USB 2.0 and the other for USB 3.0
    xhci_root_hub_t root_hubs[XHCI_RH_COUNT];

    // Maps root hub port index to the index of their virtual root hub
    uint8_t* rh_map;

    // Maps root hub port index to index relative to their virtual root hub
    uint8_t* rh_port_map;

    // Pointer to the USB Legacy Support Capability, if present.
    xhci_usb_legacy_support_cap_t* usb_legacy_support_cap;

    // device thread stuff
    thrd_t device_thread;
    xhci_slot_t* slots;

    // for command processing in xhci-device-manager.c
    list_node_t command_queue;
    mtx_t command_queue_mutex;
    completion_t command_queue_completion;

    // DMA buffers used by xhci_device_thread in xhci-device-manager.c
    uint8_t* input_context;
    mx_paddr_t input_context_phys;
    mtx_t input_context_lock;

    // for xhci_get_current_frame()
    mtx_t mfindex_mutex;
    // number of times mfindex has wrapped
    uint64_t mfindex_wrap_count;
   // time of last mfindex wrap
    mx_time_t last_mfindex_wrap;

    // VMO buffer for DCBAA and ERST array
    mx_handle_t dcbaa_erst_handle;
    mx_vaddr_t dcbaa_erst_virt;
    // VMO buffer for input context
    mx_handle_t input_context_handle;
    mx_vaddr_t input_context_virt;
    // VMO buffer for scratch pad pages
    mx_handle_t scratch_pad_pages_handle;
    mx_vaddr_t scratch_pad_pages_virt;
    // VMO buffer for scratch pad index
    mx_handle_t scratch_pad_index_handle;
    mx_vaddr_t scratch_pad_index_virt;
};

mx_status_t xhci_init(xhci_t* xhci, void* mmio);
mx_status_t xhci_endpoint_init(xhci_endpoint_t* ep, int ring_count);
void xhci_endpoint_free(xhci_endpoint_t* ep);
void xhci_start(xhci_t* xhci);
void xhci_handle_interrupt(xhci_t* xhci, bool legacy);
void xhci_post_command(xhci_t* xhci, uint32_t command, uint64_t ptr, uint32_t control_bits,
                       xhci_command_context_t* context);
void xhci_wait_bits(volatile uint32_t* ptr, uint32_t bits, uint32_t expected);

// returns monotonically increasing frame count
uint64_t xhci_get_current_frame(xhci_t* xhci);

uint8_t xhci_endpoint_index(uint8_t ep_address);

// returns index into xhci->root_hubs[], or -1 if not a root hub
int xhci_get_root_hub_index(xhci_t* xhci, uint32_t device_id);

inline bool xhci_is_root_hub(xhci_t* xhci, uint32_t device_id) {
    return xhci_get_root_hub_index(xhci, device_id) >= 0;
}

// upper layer routines in usb-xhci.c
mx_status_t xhci_add_device(xhci_t* xhci, int slot_id, int hub_address, int speed);
void xhci_remove_device(xhci_t* xhci, int slot_id);
