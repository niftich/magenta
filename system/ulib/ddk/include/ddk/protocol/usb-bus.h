// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/driver.h>
#include <magenta/compiler.h>
#include <magenta/hw/usb.h>
#include <magenta/hw/usb-hub.h>

__BEGIN_CDECLS;

typedef struct usb_bus_protocol {
    mx_status_t (*add_device)(mx_device_t* device, uint32_t device_id, uint32_t hub_id, usb_speed_t speed);
    void (*remove_device)(mx_device_t* device, uint32_t device_id);

    // Hub support
    mx_status_t (*configure_hub)(mx_device_t* dev, mx_device_t* hub_device, usb_speed_t speed,
                 usb_hub_descriptor_t* descriptor);
    mx_status_t (*hub_device_added)(mx_device_t* device, mx_device_t* hub_device, int port, usb_speed_t speed);

    mx_status_t (*hub_device_removed)(mx_device_t* device, mx_device_t* hub_device, int port);
} usb_bus_protocol_t;

__END_CDECLS;
