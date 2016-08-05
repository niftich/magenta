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

#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/binding.h>
#include <ddk/protocol/pci.h>
#include <ddk/protocol/intel-me.h>
#include <hw/pci.h>

#include <assert.h>
#include <magenta/syscalls.h>
#include <magenta/syscalls-ddk.h>
#include <magenta/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bus-protocol.h"
#include "intel-me.h"
#include "remote.h"

#define INTEL_ME_VID 0x8086
#define INTEL_ME_SPT_DID_1 0x9D3A
#define INTEL_ME_SPT_DID_2 0x9D3B
#define INTEL_ME_SPT_DID_3 0x9D3E

#define INTEL_ME_REGISTER_SIZE 20

// PCI config space special registers
#define HECI_DELIVERY_MODE(pci_config) ((volatile uint8_t *)(pci_config) + 0xA0)

// HECI_DELIVERY_MODE bits
#define HECI_DM_LOCK (1<<2)
#define HECI_DM_MASK (0x3)
#define HECI_DM_GENERATE_MSI (0x0)

#define get_intel_me_device(dev) containerof(dev, intel_me_device_t, device)

// Implement device protocol:

static mx_protocol_device_t intel_me_device_proto = {
};

// Implement transaction processing:

void intel_me_complete_txn(
        intel_me_device_t* device, int slot_num,
        mx_status_t status, mx_off_t written) {

    iotxn_t* txn = device->running_txns[slot_num];
    device->running_txns[slot_num] = NULL;

    txn->ops->complete(txn, status, written);

    // hit the worker thread to do the next txn
    mxr_completion_signal(&device->worker_completion);
}

static void start_txn(
        intel_me_device_t* device, uint8_t slot_num, iotxn_t* txn) {

    assert(!device->running_txns[slot_num]);
    device->running_txns[slot_num] = txn;

    mx_status_t status = intel_me_send_message(
            device,
            slot_num);
    if (status != NO_ERROR) {
        intel_me_complete_txn(device, slot_num, status, 0);
        return;
    }
}

static int iotxn_worker_thread(void* arg) {
    intel_me_device_t* device = get_intel_me_device(arg);
    list_node_t* node;
    for (;;) {
        printf("LOOKING FOR OPEN SLOT\n");
        // Look for an open transaction slot
        unsigned int open_slot;
        for (open_slot = 0; open_slot < MAX_RUNNING_TXNS; ++open_slot) {
            if (!device->running_txns[open_slot]) {
                break;
            }
        }

        if (open_slot != MAX_RUNNING_TXNS && intel_me_ready_for_send(device)) {
            // run the next command if not busy
            mxr_mutex_lock(&device->txn_list_lock);
            node = list_remove_head(&device->txn_list);
            mxr_mutex_unlock(&device->txn_list_lock);

            if (node) {
                printf("STARTING TXN\n");
                start_txn(device, open_slot, containerof(node, iotxn_t, node));
            }
        }

        printf("WORKER SLEEPING\n");
        // wait here until more commands are queued, or the bus is ready for
        // another command to be sent
        mxr_completion_wait(&device->worker_completion, MX_TIME_INFINITE);
        mxr_completion_reset(&device->worker_completion);
    }
    return 0;
}

static int irq_thread(void* arg) {
    intel_me_device_t* device = arg;
    intel_me_set_irq_enable(device, true);
    for (;;) {
        mx_status_t r;
        if ((r = mx_pci_interrupt_wait(device->irq_handle)) < 0) {
            printf("Intel ME: irq wait failed? %d\n", r);
            break;
        }
        printf("INTERRUPT RECEIVED\n");
        intel_me_clear_irq(device);

        if (intel_me_pending_data(device)) {
            printf("RECEIVING DATA\n");
            // TODO: offload this from the irq thread
            intel_me_recv_message(device);
        }

        if (intel_me_ready_for_send(device)) {
            printf("READY FOR SENDING\n");
            mxr_completion_signal(&device->worker_completion);
        }
    }
    return 0;
}

void intel_me_iotxn_queue(
        intel_me_device_t* device,
        intel_me_remote_device_t* remote,
        iotxn_t* txn) {

    if (txn->protocol != MX_PROTOCOL_INTEL_ME) {
        txn->ops->complete(txn, ERR_INVALID_ARGS, 0);
        return;
    }

    real_iotxn_data_t* pdata = iotxn_pdata(txn, real_iotxn_data_t);
    pdata->remote_addr = remote->me_addr;

    printf("ENQUEUING TRANSACTION\n");

    // put the request on the queue
    mxr_mutex_lock(&device->txn_list_lock);
    list_add_tail(&device->txn_list, &txn->node);
    mxr_mutex_unlock(&device->txn_list_lock);

    // hit the worker thread
    mxr_completion_signal(&device->worker_completion);
}

// Implement driver object:

static mx_status_t setup_irq(intel_me_device_t* device,
                               mx_device_t* pci_dev,
                               pci_protocol_t* pci_proto) {
    // Register for MSI delivery to our driver
    mx_status_t status = pci_proto->set_irq_mode(
            pci_dev, MX_PCIE_IRQ_MODE_LEGACY, 1);
    if (status) {
        return status;
    }

    device->irq_handle = pci_proto->map_interrupt(pci_dev, 0);
    if (device->irq_handle < 0) {
        return status;
    }

    status = mxr_thread_create(
            irq_thread, device, "intel-me-irq-thread", &device->irq_thread);
    if (status) {
        return status;
    }
    mxr_thread_detach(device->irq_thread);

    return NO_ERROR;
}

static mx_status_t setup_iotxn_thread(intel_me_device_t* device) {
    mx_status_t status = mxr_thread_create(
            iotxn_worker_thread, device, "intel-me-worker-thread",
            &device->worker_thread);
    if (status) {
        return status;
    }
    mxr_thread_detach(device->worker_thread);

    return NO_ERROR;
}

static mx_status_t intel_me_bind(mx_driver_t* drv, mx_device_t* dev) {
    pci_protocol_t* pci;
    if (device_get_protocol(dev, MX_PROTOCOL_PCI, (void**)&pci))
        return ERR_NOT_SUPPORTED;

    mx_status_t status = pci->claim_device(dev);
    if (status < 0)
        return status;

    // map resources and initialize the device
    intel_me_device_t* device = calloc(1, sizeof(intel_me_device_t));
    if (!device)
        return ERR_NO_MEMORY;

    device->bus_lock = MXR_MUTEX_INIT;
    device->txn_list_lock = MXR_MUTEX_INIT;
    device->worker_completion = MXR_COMPLETION_INIT;
    list_initialize(&device->txn_list);

    const pci_config_t* pci_config;
    mx_handle_t cfg_handle = pci->get_config(dev, &pci_config);
    if (cfg_handle < 0) {
        status = cfg_handle;
        goto fail;
    }
    uint16_t device_id = pci_config->device_id;

    volatile uint8_t *heci_dm_reg = HECI_DELIVERY_MODE(pci_config);
    // Make sure that the HECI delivery mode isn't locked, or failing that
    // that it is locked to MSI mode (which we plan to use).
    if ((*heci_dm_reg & HECI_DM_LOCK) &&
        (*heci_dm_reg & HECI_DM_MASK) != HECI_DM_GENERATE_MSI) {

        goto fail;
    }

    // map register window
    device->regs_handle = pci->map_mmio(dev, 0, MX_CACHE_POLICY_UNCACHED_DEVICE,
                                        (void**)&device->regs,
                                        &device->regs_size);
    if (device->regs_handle < 0) {
        status = device->regs_handle;
        goto fail;
    }

    if (device->regs_size < INTEL_ME_REGISTER_SIZE) {
        status = ERR_NOT_SUPPORTED;
        goto fail;
    }

    // create and add the device
    char name[20];
    snprintf(name, sizeof(name), "intel-me-%04x", device_id);
    status = device_init(&device->device, drv, name, &intel_me_device_proto);
    if (status) {
        goto fail;
    }

    status = device_add(&device->device, dev);
    if (status) {
        goto fail;
    }

    // Set the interrupt delivery mode to MSI (versus SCI or SMI), so that
    // this driver can manage it.
    if ((*heci_dm_reg & HECI_DM_MASK) != HECI_DM_GENERATE_MSI) {
#if 0 // Disabled due to lack of PCI config space write privileges
        uint8_t heci_dm = *heci_dm_reg;
        heci_dm &= ~HECI_DM_MASK;
        heci_dm |= HECI_DM_GENERATE_MSI;
        *heci_dm_reg = heci_dm;
#endif
        goto unregister_and_fail;
    }

    status = setup_irq(device, dev, pci);
    if (status) {
        goto unregister_and_fail;
    }

    status = setup_iotxn_thread(device);
    if (status) {
        goto unregister_and_fail;
    }

    intel_me_reset(device);

    // TODO(teisenbe): Do real discovery on the ME bus
    if (device_id == INTEL_ME_SPT_DID_3) {
        // Create a remote device for Intel PreciseTouch
        intel_me_remote_device_t* precise_touch;
        // If we fail to create the remote device, ignore it
        intel_me_remote_device_create(device,
                                      INTEL_ME_REMOTE_ADDR_PRECISE_TOUCH,
                                      &precise_touch);

        intel_me_test(device, precise_touch);
    }

    return NO_ERROR;
unregister_and_fail:
    device_remove(&device->device);
fail:
    if (device->irq_handle > 0) {
        mx_handle_close(device->irq_handle);
    }
    if (device->regs_handle > 0) {
        mx_handle_close(device->regs_handle);
    }
    if (cfg_handle > 0) {
        mx_handle_close(cfg_handle);
    }
    free(device);
    return status;
}

static mx_bind_inst_t binding[] = {
    BI_ABORT_IF(NE, BIND_PROTOCOL, MX_PROTOCOL_PCI),
    BI_ABORT_IF(NE, BIND_PCI_VID, INTEL_ME_VID),
    BI_MATCH_IF(EQ, BIND_PCI_DID, INTEL_ME_SPT_DID_1),
    BI_MATCH_IF(EQ, BIND_PCI_DID, INTEL_ME_SPT_DID_2),
    BI_MATCH_IF(EQ, BIND_PCI_DID, INTEL_ME_SPT_DID_3),
};

mx_driver_t _driver_intel_me BUILTIN_DRIVER = {
    .name = "intel_me",
    .ops = {
        .bind = intel_me_bind,
    },
    .binding = binding,
    .binding_size = sizeof(binding),
};
