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

#include <stdlib.h>
#include <stdio.h>

#include "remote.h"

#define get_intel_me_remote_device(dev) \
        containerof(dev, intel_me_remote_device_t, device)

static void iotxn_complete_cb(iotxn_t* txn) {
    iotxn_t *orig_txn = txn->context;

    orig_txn->ops->complete(orig_txn, txn->status, txn->actual);
    txn->ops->release(txn);
}

static void intel_me_remote_iotxn_queue(mx_device_t* dev, iotxn_t* txn) {
    intel_me_remote_device_t *device = get_intel_me_remote_device(dev);

    if (txn->protocol != MX_PROTOCOL_INTEL_ME) {
        txn->ops->complete(txn, ERR_INVALID_ARGS, 0);
        return;
    }

    // Create a new transaction so we can put internal data in the protocol
    // data section
    iotxn_t *new_txn;
    mx_status_t status = txn->ops->clone(txn, &new_txn, 0);
    if (status != NO_ERROR) {
        txn->ops->complete(txn, status, 0);
        return;
    }
    new_txn->context = txn;

    intel_me_iotxn_queue(device->bus, device, txn);
}

static mx_protocol_device_t intel_me_remote_device_proto = {
    .iotxn_queue = intel_me_remote_iotxn_queue,
};

mx_status_t intel_me_remote_device_create(intel_me_device_t* bus,
                                          intel_me_remote_address_t addr,
                                          intel_me_remote_device_t** remote) {
    *remote = NULL;

    intel_me_remote_device_t* device = calloc(1, sizeof(intel_me_remote_device_t));
    if (!device) {
        return ERR_NO_MEMORY;
    }

    device->me_addr = addr;
    device->bus = bus;

    char name[MX_DEVICE_NAME_MAX];
    snprintf(name, sizeof(name), "%s-%02x", bus->device.name, addr);
    mx_status_t status = device_init(&device->device, bus->device.driver, name,
                                     &intel_me_remote_device_proto);
    if (status != NO_ERROR) {
        goto fail;
    }

    device->props[0] =
            (mx_device_prop_t){ BIND_PROTOCOL, 0, MX_PROTOCOL_INTEL_ME };
    device->props[1] = (mx_device_prop_t){ BIND_INTEL_ME_ADDR, 0, addr };
    device->device.props = device->props;
    device->device.prop_count = countof(device->props);

    status = device_add(&device->device, &bus->device);
    if (status) {
        goto fail;
    }

    *remote = device;
    return NO_ERROR;

fail:
    free(device);
    return status;
}
