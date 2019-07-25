// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <hw/reg.h>
#include <magenta/types.h>
#include <magenta/syscalls.h>
#include <magenta/process.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>
#include <unistd.h>

#include "xhci.h"
#include "xhci-device-manager.h"
#include "xhci-root-hub.h"
#include "xhci-transfer.h"

//#define TRACE 1
#include "xhci-debug.h"

#define PAGE_ROUNDUP(x) ((x + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1))

uint8_t xhci_endpoint_index(uint8_t ep_address) {
    if (ep_address == 0) return 0;
    uint32_t index = 2 * (ep_address & ~USB_ENDPOINT_DIR_MASK);
    if ((ep_address & USB_ENDPOINT_DIR_MASK) == USB_ENDPOINT_OUT)
        index--;
    return index;
}

// returns index into xhci->root_hubs[], or -1 if not a root hub
int xhci_get_root_hub_index(xhci_t* xhci, uint32_t device_id) {
    // regular devices have IDs 1 through xhci->max_slots
    // root hub IDs start at xhci->max_slots + 1
    int index = device_id - (xhci->max_slots + 1);
    if (index < 0 || index >= XHCI_RH_COUNT) return -1;
    return index;
}

static void xhci_read_extended_caps(xhci_t* xhci, void* mmio, volatile uint32_t* hccparams1) {
    uint32_t offset = XHCI_GET_BITS32(hccparams1, HCCPARAMS1_EXT_CAP_PTR_START,
                                      HCCPARAMS1_EXT_CAP_PTR_BITS);
    if (!offset) return;
    // offset is 32-bit words from MMIO base
    uint32_t* cap_ptr = (uint32_t *)(mmio + (offset << 2));

    while (cap_ptr) {
        uint32_t cap_id = XHCI_GET_BITS32(cap_ptr, EXT_CAP_CAPABILITY_ID_START,
                                          EXT_CAP_CAPABILITY_ID_BITS);

        if (cap_id == EXT_CAP_SUPPORTED_PROTOCOL) {
            uint32_t rev_major = XHCI_GET_BITS32(cap_ptr, EXT_CAP_SP_REV_MAJOR_START,
                                                 EXT_CAP_SP_REV_MAJOR_BITS);
#if (TRACE == 1)
            uint32_t rev_minor = XHCI_GET_BITS32(cap_ptr, EXT_CAP_SP_REV_MINOR_START,
                                                 EXT_CAP_SP_REV_MINOR_BITS);
            printf("EXT_CAP_SUPPORTED_PROTOCOL %d.%d\n", rev_major, rev_minor);

            uint32_t psic = XHCI_GET_BITS32(&cap_ptr[2], EXT_CAP_SP_PSIC_START,
                                            EXT_CAP_SP_PSIC_BITS);
#endif
            // psic = count of PSI registers
            uint32_t compat_port_offset = XHCI_GET_BITS32(&cap_ptr[2],
                                                          EXT_CAP_SP_COMPAT_PORT_OFFSET_START,
                                                          EXT_CAP_SP_COMPAT_PORT_OFFSET_BITS);
            uint32_t compat_port_count = XHCI_GET_BITS32(&cap_ptr[2],
                                                         EXT_CAP_SP_COMPAT_PORT_COUNT_START,
                                                         EXT_CAP_SP_COMPAT_PORT_COUNT_BITS);

            xprintf("compat_port_offset: %d compat_port_count: %d psic: %d\n", compat_port_offset,
                   compat_port_count, psic);

            int rh_index;
            if (rev_major == 3) {
                rh_index = XHCI_RH_USB_3;
            } else if (rev_major == 2) {
                rh_index = XHCI_RH_USB_2;
            } else {
                printf("unsupported rev_major in XHCI extended capabilities\n");
                rh_index = -1;
            }
            for (off_t i = 0; i < compat_port_count; i++) {
                off_t index = compat_port_offset + i - 1;
                if (index >= xhci->rh_num_ports) {
                    printf("port index out of range in xhci_read_extended_caps\n");
                    break;
                }
                xhci->rh_map[index] = rh_index;
            }

#if (TRACE == 1)
            uint32_t* psi = &cap_ptr[4];
            for (uint32_t i = 0; i < psic; i++, psi++) {
                uint32_t psiv = XHCI_GET_BITS32(psi, EXT_CAP_SP_PSIV_START, EXT_CAP_SP_PSIV_BITS);
                uint32_t psie = XHCI_GET_BITS32(psi, EXT_CAP_SP_PSIE_START, EXT_CAP_SP_PSIE_BITS);
                uint32_t plt = XHCI_GET_BITS32(psi, EXT_CAP_SP_PLT_START, EXT_CAP_SP_PLT_BITS);
                uint32_t psim = XHCI_GET_BITS32(psi, EXT_CAP_SP_PSIM_START, EXT_CAP_SP_PSIM_BITS);
                printf("PSI[%d] psiv: %d psie: %d plt: %d psim: %d\n", i, psiv, psie, plt, psim);
            }
#endif
        } else if (cap_id == EXT_CAP_USB_LEGACY_SUPPORT) {
            xhci->usb_legacy_support_cap = (xhci_usb_legacy_support_cap_t*)cap_ptr;
        }

        // offset is 32-bit words from cap_ptr
        offset = XHCI_GET_BITS32(cap_ptr, EXT_CAP_NEXT_PTR_START, EXT_CAP_NEXT_PTR_BITS);
        cap_ptr = (offset ? cap_ptr + offset : NULL);
    }
}

static mx_status_t xhci_claim_ownership(xhci_t* xhci) {
    xhci_usb_legacy_support_cap_t* cap = xhci->usb_legacy_support_cap;
    if (cap == NULL) {
        return NO_ERROR;
    }

    // The XHCI spec defines this handoff protocol.  We need to wait at most one
    // second for the BIOS to respond.
    //
    // Note that bios_owned_sem and os_owned_sem are adjacent 1-byte fields, so
    // must be written to as single bytes to prevent the OS from modifying the
    // BIOS semaphore.  Additionally, all bits besides bit 0 in the OS semaphore
    // are RsvdP, so we need to preserve them on modification.
    cap->os_owned_sem |= 1;
    mx_time_t now = mx_time_get(MX_CLOCK_MONOTONIC);
    mx_time_t deadline = now + MX_SEC(1);
    while ((cap->bios_owned_sem & 1) && now < deadline) {
        mx_nanosleep(mx_deadline_after(MX_MSEC(10)));
        now = mx_time_get(MX_CLOCK_MONOTONIC);
    }

    if (cap->bios_owned_sem & 1) {
        cap->os_owned_sem &= ~1;
        return ERR_TIMED_OUT;
    }
    return NO_ERROR;
}

static mx_status_t xhci_vmo_init(size_t size, mx_handle_t* out_handle, mx_vaddr_t* out_virt,
                                 bool contiguous) {
    mx_status_t status;
    mx_handle_t handle;

    if (contiguous) {
        status = mx_vmo_create_contiguous(get_root_resource(), size, 0, &handle);
    } else {
        status = mx_vmo_create(size, 0, &handle);
    }
    if (status != NO_ERROR) {
        printf("xhci_vmo_init: vmo_create failed: %d\n", status);
        return status;
    }

    if (!contiguous) {
        // needs to be done before MX_VMO_OP_LOOKUP for non-contiguous VMOs
        status = mx_vmo_op_range(handle, MX_VMO_OP_COMMIT, 0, size, NULL, 0);
        if (status != NO_ERROR) {
            printf("xhci_vmo_init: mx_vmo_op_range(MX_VMO_OP_COMMIT) failed %d\n", status);
            mx_handle_close(handle);
            return status;
        }
    }

    status = mx_vmar_map(mx_vmar_root_self(), 0, handle, 0, size,
                         MX_VM_FLAG_PERM_READ | MX_VM_FLAG_PERM_WRITE, out_virt);
    if (status != NO_ERROR) {
        printf("xhci_vmo_init: mx_vmar_map failed: %d\n", status);
        mx_handle_close(handle);
        return status;
    }

    *out_handle = handle;
    return NO_ERROR;
}

static void xhci_vmo_release(mx_handle_t handle, mx_vaddr_t virt) {
    uint64_t size;
    mx_vmo_get_size(handle, &size);
    mx_vmar_unmap(mx_vmar_root_self(), virt, size);
    mx_handle_close(handle);
}

mx_status_t xhci_init(xhci_t* xhci, void* mmio) {
    mx_status_t result = NO_ERROR;
    mx_paddr_t* phys_addrs = NULL;

    list_initialize(&xhci->command_queue);
    mtx_init(&xhci->command_ring_lock, mtx_plain);
    mtx_init(&xhci->command_queue_mutex, mtx_plain);
    mtx_init(&xhci->mfindex_mutex, mtx_plain);
    mtx_init(&xhci->input_context_lock, mtx_plain);
    completion_reset(&xhci->command_queue_completion);

    xhci->cap_regs = (xhci_cap_regs_t*)mmio;
    xhci->op_regs = (xhci_op_regs_t*)((uint8_t*)xhci->cap_regs + xhci->cap_regs->length);
    xhci->doorbells = (uint32_t*)((uint8_t*)xhci->cap_regs + xhci->cap_regs->dboff);
    xhci->runtime_regs = (xhci_runtime_regs_t*)((uint8_t*)xhci->cap_regs + xhci->cap_regs->rtsoff);
    volatile uint32_t* hcsparams1 = &xhci->cap_regs->hcsparams1;
    volatile uint32_t* hcsparams2 = &xhci->cap_regs->hcsparams2;
    volatile uint32_t* hccparams1 = &xhci->cap_regs->hccparams1;
    volatile uint32_t* hccparams2 = &xhci->cap_regs->hccparams2;

    xhci->max_slots = XHCI_GET_BITS32(hcsparams1, HCSPARAMS1_MAX_SLOTS_START,
                                      HCSPARAMS1_MAX_SLOTS_BITS);
    xhci->max_interruptors = XHCI_GET_BITS32(hcsparams1, HCSPARAMS1_MAX_INTRS_START,
                                             HCSPARAMS1_MAX_INTRS_BITS);
    xhci->rh_num_ports = XHCI_GET_BITS32(hcsparams1, HCSPARAMS1_MAX_PORTS_START,
                                         HCSPARAMS1_MAX_PORTS_BITS);
    xhci->context_size = (XHCI_READ32(hccparams1) & HCCPARAMS1_CSZ ? 64 : 32);
    xhci->large_esit = !!(XHCI_READ32(hccparams2) & HCCPARAMS2_LEC);

    uint32_t scratch_pad_bufs = XHCI_GET_BITS32(hcsparams2, HCSPARAMS2_MAX_SBBUF_HI_START,
                                                HCSPARAMS2_MAX_SBBUF_HI_BITS);
    scratch_pad_bufs <<= HCSPARAMS2_MAX_SBBUF_LO_BITS;
    scratch_pad_bufs |= XHCI_GET_BITS32(hcsparams2, HCSPARAMS2_MAX_SBBUF_LO_START,
                                        HCSPARAMS2_MAX_SBBUF_LO_BITS);
    xhci->page_size = XHCI_READ32(&xhci->op_regs->pagesize) << 12;

    // allocate array to hold our slots
    // add 1 to allow 1-based indexing of slots
    xhci->slots = (xhci_slot_t*)calloc(xhci->max_slots + 1, sizeof(xhci_slot_t));
    if (!xhci->slots) {
        result = ERR_NO_MEMORY;
        goto fail;
    }

    xhci->rh_map = (uint8_t *)calloc(xhci->rh_num_ports, sizeof(uint8_t));
    if (!xhci->rh_map) {
        result = ERR_NO_MEMORY;
        goto fail;
    }
    xhci->rh_port_map = (uint8_t *)calloc(xhci->rh_num_ports, sizeof(uint8_t));
    if (!xhci->rh_port_map) {
        result = ERR_NO_MEMORY;
        goto fail;
    }
    xhci_read_extended_caps(xhci, mmio, hccparams1);

    // We need to claim before we write to any other registers on the
    // controller, but after we've read the extended capabilities.
    result = xhci_claim_ownership(xhci);
    if (result != NO_ERROR) {
        printf("xhci_claim_ownership failed\n");
        goto fail;
    }

    // Allocate DMA memory for various things
    result = xhci_vmo_init(PAGE_SIZE, &xhci->dcbaa_erst_handle, &xhci->dcbaa_erst_virt, false);
    if (result != NO_ERROR) {
        printf("xhci_vmo_init failed for xhci->dcbaa_erst_handle\n");
        goto fail;
    }
    result = xhci_vmo_init(PAGE_SIZE, &xhci->input_context_handle, &xhci->input_context_virt, false);
    if (result != NO_ERROR) {
        printf("xhci_vmo_init failed for xhci->input_context_handle\n");
        goto fail;
    }

    if (scratch_pad_bufs > 0) {
        size_t scratch_pad_pages_size = scratch_pad_bufs * xhci->page_size;
        result = xhci_vmo_init(scratch_pad_pages_size, &xhci->scratch_pad_pages_handle,
                               &xhci->scratch_pad_pages_virt, xhci->page_size > PAGE_SIZE);
        if (result != NO_ERROR) {
            printf("xhci_vmo_init failed for xhci->scratch_pad_pages_handle\n");
            goto fail;
        }
        size_t scratch_pad_index_size = PAGE_ROUNDUP(scratch_pad_bufs * sizeof(uint64_t));
        result = xhci_vmo_init(scratch_pad_index_size, &xhci->scratch_pad_index_handle,
                               &xhci->scratch_pad_index_virt, scratch_pad_index_size > PAGE_SIZE);
        if (result != NO_ERROR) {
            printf("xhci_vmo_init failed for xhci->scratch_pad_index_handle\n");
            goto fail;
        }
    }

    // set up DCBAA, ERST array and input context
    xhci->dcbaa = (uint64_t *)xhci->dcbaa_erst_virt;
    result = mx_vmo_op_range(xhci->dcbaa_erst_handle, MX_VMO_OP_LOOKUP, 0, PAGE_SIZE,
                             &xhci->dcbaa_phys, sizeof(xhci->dcbaa_phys));
    if (result != NO_ERROR) {
        printf("mx_vmo_op_range failed for xhci->dcbaa_erst_handle\n");
        goto fail;
    }
    xhci->input_context = (uint8_t *)xhci->input_context_virt;
    result = mx_vmo_op_range(xhci->input_context_handle, MX_VMO_OP_LOOKUP, 0, PAGE_SIZE,
                             &xhci->input_context_phys, sizeof(xhci->input_context_phys));
    if (result != NO_ERROR) {
        printf("mx_vmo_op_range failed for xhci->input_context_handle\n");
        goto fail;
    }

    // DCBAA can only be 256 * sizeof(uint64_t) = 2048 bytes, so we have room for ERST array after DCBAA
    mx_off_t erst_offset = 256 * sizeof(uint64_t);
    xhci->erst_arrays[0] = (void *)xhci->dcbaa + erst_offset;
    xhci->erst_arrays_phys[0] = xhci->dcbaa_phys + erst_offset;

    if (scratch_pad_bufs > 0) {
        uint64_t* scratch_pad_index = (uint64_t *)xhci->scratch_pad_index_virt;
        off_t offset = 0;
        for (uint32_t i = 0; i < scratch_pad_bufs; i++) {
            mx_paddr_t scratch_pad_phys;
            result = mx_vmo_op_range(xhci->scratch_pad_pages_handle, MX_VMO_OP_LOOKUP, offset,
                                     PAGE_SIZE, &scratch_pad_phys, sizeof(scratch_pad_phys));
            if (result != NO_ERROR) {
                printf("mx_vmo_op_range failed for xhci->scratch_pad_pages_handle\n");
                goto fail;
            }
            scratch_pad_index[i] = scratch_pad_phys;
            offset += xhci->page_size;
        }

        mx_paddr_t scratch_pad_index_phys;
        result = mx_vmo_op_range(xhci->scratch_pad_index_handle, MX_VMO_OP_LOOKUP, 0, PAGE_SIZE,
                                  &scratch_pad_index_phys, sizeof(scratch_pad_index_phys));
        if (result != NO_ERROR) {
            printf("mx_vmo_op_range failed for xhci->scratch_pad_index_handle\n");
            goto fail;
        }

        xhci->dcbaa[0] = scratch_pad_index_phys;
    } else {
        xhci->dcbaa[0] = 0;
    }

    result = xhci_transfer_ring_init(&xhci->command_ring, COMMAND_RING_SIZE);
    if (result != NO_ERROR) {
        printf("xhci_command_ring_init failed\n");
        goto fail;
    }

    result = xhci_event_ring_init(xhci, 0, EVENT_RING_SIZE);
    if (result != NO_ERROR) {
        printf("xhci_event_ring_init failed\n");
        goto fail;
    }

    // initialize virtual root hub devices
    for (int i = 0; i < XHCI_RH_COUNT; i++) {
        result = xhci_root_hub_init(xhci, i);
        if (result != NO_ERROR) goto fail;
    }

    free(phys_addrs);

    return NO_ERROR;

fail:
    for (int i = 0; i < XHCI_RH_COUNT; i++) {
        xhci_root_hub_free(&xhci->root_hubs[i]);
    }
    free(xhci->rh_map);
    free(xhci->rh_port_map);
    xhci_event_ring_free(xhci, 0);
    xhci_transfer_ring_free(&xhci->command_ring);
    xhci_vmo_release(xhci->dcbaa_erst_handle, xhci->dcbaa_erst_virt);
    xhci_vmo_release(xhci->input_context_handle, xhci->input_context_virt);
    xhci_vmo_release(xhci->scratch_pad_pages_handle, xhci->scratch_pad_pages_virt);
    xhci_vmo_release(xhci->scratch_pad_index_handle, xhci->scratch_pad_index_virt);
    free(phys_addrs);
    free(xhci->slots);
    return result;
}

mx_status_t xhci_endpoint_init(xhci_endpoint_t* ep, int ring_count) {
    mx_status_t status = xhci_transfer_ring_init(&ep->transfer_ring, ring_count);
    if (status != NO_ERROR) return status;

    mtx_init(&ep->lock, mtx_plain);
    list_initialize(&ep->queued_txns);
    list_initialize(&ep->pending_txns);
    ep->current_txn = NULL;
    return NO_ERROR;
}

void xhci_endpoint_free(xhci_endpoint_t* ep) {
    free(ep->transfer_state);
    ep->transfer_state = NULL;
}

static void xhci_update_erdp(xhci_t* xhci, int interruptor) {
    xhci_event_ring_t* er = &xhci->event_rings[interruptor];
    xhci_intr_regs_t* intr_regs = &xhci->runtime_regs->intr_regs[interruptor];

    uint64_t erdp = xhci_event_ring_current_phys(er);
    erdp |= ERDP_EHB; // clear event handler busy
    XHCI_WRITE64(&intr_regs->erdp, erdp);
}

static void xhci_interruptor_init(xhci_t* xhci, int interruptor) {
    xhci_intr_regs_t* intr_regs = &xhci->runtime_regs->intr_regs[interruptor];

    xhci_update_erdp(xhci, interruptor);

    XHCI_SET32(&intr_regs->iman, IMAN_IE, IMAN_IE);
    XHCI_SET32(&intr_regs->erstsz, ERSTSZ_MASK, ERST_ARRAY_SIZE);
    XHCI_WRITE64(&intr_regs->erstba, xhci->erst_arrays_phys[interruptor]);
}

void xhci_wait_bits(volatile uint32_t* ptr, uint32_t bits, uint32_t expected) {
    uint32_t value = XHCI_READ32(ptr);
    while ((value & bits) != expected) {
        usleep(1000);
        value = XHCI_READ32(ptr);
    }
}

void xhci_start(xhci_t* xhci) {
    volatile uint32_t* usbcmd = &xhci->op_regs->usbcmd;
    volatile uint32_t* usbsts = &xhci->op_regs->usbsts;

    xhci_wait_bits(usbsts, USBSTS_CNR, 0);

    // stop controller
    XHCI_SET32(usbcmd, USBCMD_RS, 0);
    // wait until USBSTS_HCH signals we stopped
    xhci_wait_bits(usbsts, USBSTS_HCH, USBSTS_HCH);

    XHCI_SET32(usbcmd, USBCMD_HCRST, USBCMD_HCRST);
    xhci_wait_bits(usbcmd, USBCMD_HCRST, 0);
    xhci_wait_bits(usbsts, USBSTS_CNR, 0);

    // setup operational registers
    xhci_op_regs_t* op_regs = xhci->op_regs;
    // initialize command ring
    uint64_t crcr = xhci_transfer_ring_start_phys(&xhci->command_ring);
    crcr |= CRCR_RCS;
    XHCI_WRITE64(&op_regs->crcr, crcr);

    XHCI_WRITE64(&op_regs->dcbaap, xhci->dcbaa_phys);
    XHCI_SET_BITS32(&op_regs->config, CONFIG_MAX_SLOTS_ENABLED_START,
                    CONFIG_MAX_SLOTS_ENABLED_BITS, xhci->max_slots);

    // initialize interruptor (only using one for now)
    xhci_interruptor_init(xhci, 0);

    // start the controller with interrupts and mfindex wrap events enabled
    uint32_t start_flags = USBCMD_RS | USBCMD_INTE | USBCMD_EWE;
    XHCI_SET32(usbcmd, start_flags, start_flags);
    xhci_wait_bits(usbsts, USBSTS_HCH, 0);

    xhci_start_device_thread(xhci);
}

void xhci_post_command(xhci_t* xhci, uint32_t command, uint64_t ptr, uint32_t control_bits,
                       xhci_command_context_t* context) {
    // FIXME - check that command ring is not full?

    mtx_lock(&xhci->command_ring_lock);

    xhci_transfer_ring_t* cr = &xhci->command_ring;
    xhci_trb_t* trb = cr->current;
    int index = trb - cr->start;
    xhci->command_contexts[index] = context;

    XHCI_WRITE64(&trb->ptr, ptr);
    XHCI_WRITE32(&trb->status, 0);
    trb_set_control(trb, command, control_bits);

    xhci_increment_ring(cr);

    XHCI_WRITE32(&xhci->doorbells[0], 0);

    mtx_unlock(&xhci->command_ring_lock);
}

static void xhci_handle_command_complete_event(xhci_t* xhci, xhci_trb_t* event_trb) {
    xhci_trb_t* command_trb = xhci_read_trb_ptr(&xhci->command_ring, event_trb);
    uint32_t cc = XHCI_GET_BITS32(&event_trb->status, EVT_TRB_CC_START, EVT_TRB_CC_BITS);
    xprintf("xhci_handle_command_complete_event slot_id: %d command: %d cc: %d\n",
            (event_trb->control >> TRB_SLOT_ID_START), trb_get_type(command_trb), cc);

    int index = command_trb - xhci->command_ring.start;
    mtx_lock(&xhci->command_ring_lock);
    xhci_command_context_t* context = xhci->command_contexts[index];
    xhci->command_contexts[index] = NULL;
    mtx_unlock(&xhci->command_ring_lock);

    context->callback(context->data, cc, command_trb, event_trb);
}

static void xhci_handle_mfindex_wrap(xhci_t* xhci) {
    mtx_lock(&xhci->mfindex_mutex);
    xhci->mfindex_wrap_count++;
    xhci->last_mfindex_wrap = mx_time_get(MX_CLOCK_MONOTONIC);
    mtx_unlock(&xhci->mfindex_mutex);
}

uint64_t xhci_get_current_frame(xhci_t* xhci) {
    mtx_lock(&xhci->mfindex_mutex);

    uint32_t mfindex = XHCI_READ32(&xhci->runtime_regs->mfindex) & ((1 << XHCI_MFINDEX_BITS) - 1);
    uint64_t wrap_count = xhci->mfindex_wrap_count;
    // try to detect race condition where mfindex has wrapped but we haven't processed wrap event yet
    if (mfindex < 500) {
        if (mx_time_get(MX_CLOCK_MONOTONIC) - xhci->last_mfindex_wrap > MX_MSEC(1000)) {
            xprintf("woah, mfindex wrapped before we got the event!\n");
            wrap_count++;
        }
    }
    mtx_unlock(&xhci->mfindex_mutex);

    // shift three to convert from 125us microframes to 1ms frames
    return ((wrap_count * (1 << XHCI_MFINDEX_BITS)) + mfindex) >> 3;
}

static void xhci_handle_events(xhci_t* xhci, int interruptor) {
    xhci_event_ring_t* er = &xhci->event_rings[interruptor];

    // process all TRBs with cycle bit matching our CCS
    while ((XHCI_READ32(&er->current->control) & TRB_C) == er->ccs) {
        uint32_t type = trb_get_type(er->current);
        switch (type) {
        case TRB_EVENT_COMMAND_COMP:
            xhci_handle_command_complete_event(xhci, er->current);
            break;
        case TRB_EVENT_PORT_STATUS_CHANGE:
            // ignore, these are dealt with in xhci_handle_interrupt() below
            break;
        case TRB_EVENT_TRANSFER:
            xhci_handle_transfer_event(xhci, er->current);
            break;
        case TRB_EVENT_MFINDEX_WRAP:
            xhci_handle_mfindex_wrap(xhci);
            break;
        default:
            printf("xhci_handle_events: unhandled event type %d\n", type);
            break;
        }

        er->current++;
        if (er->current == er->end) {
            er->current = er->start;
            er->ccs ^= TRB_C;
        }
        xhci_update_erdp(xhci, interruptor);
    }
}

void xhci_handle_interrupt(xhci_t* xhci, bool legacy) {
    volatile uint32_t* usbsts = &xhci->op_regs->usbsts;
    const int interruptor = 0;

    uint32_t status = XHCI_READ32(usbsts);
    uint32_t clear = status & USBSTS_CLEAR_BITS;
    XHCI_WRITE32(usbsts, clear);

    // If we are in legacy IRQ mode, clear the IP (Interrupt Pending) bit
    // from the IMAN register of our interrupter.
    if (legacy) {
        xhci_intr_regs_t* intr_regs = &xhci->runtime_regs->intr_regs[interruptor];
        XHCI_SET32(&intr_regs->iman, IMAN_IP, IMAN_IP);
    }

    if (status & USBSTS_EINT) {
        xhci_handle_events(xhci, interruptor);
    }
    if (status & USBSTS_PCD) {
        xhci_handle_root_hub_change(xhci);
    }
}
