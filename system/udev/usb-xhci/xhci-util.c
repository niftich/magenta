// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "xhci-util.h"

static void xhci_sync_command_callback(void* data, uint32_t cc, xhci_trb_t* command_trb,
                                         xhci_trb_t* event_trb) {
    xhci_sync_command_t* command = (xhci_sync_command_t*)data;
    command->status = XHCI_READ32(&event_trb->status);
    command->control = XHCI_READ32(&event_trb->control);
    completion_signal(&command->completion);
}

void xhci_sync_command_init(xhci_sync_command_t* command) {
    completion_reset(&command->completion);
    command->context.callback = xhci_sync_command_callback;
    command->context.data = command;
}

// returns condition code
int xhci_sync_command_wait(xhci_sync_command_t* command) {
    completion_wait(&command->completion, MX_TIME_INFINITE);

    return (command->status & XHCI_MASK(EVT_TRB_CC_START, EVT_TRB_CC_BITS)) >> EVT_TRB_CC_START;
}
