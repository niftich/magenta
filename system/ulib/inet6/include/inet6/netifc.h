// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

// setup networking
int netifc_open(void);

// process inbound packet(s)
int netifc_poll(void);

// return nonzero if interface exists
int netifc_active(void);

// shut down networking
void netifc_close(void);

// set a timer to expire after ms milliseconds
void netifc_set_timer(uint32_t ms);

// returns true once the timer has expired
int netifc_timer_expired(void);

// write packet to network
// packet is discarded if too large, too small, network offline, etc
void netifc_send(const void* data, size_t len);

void netifc_recv(void* data, size_t len);

void netifc_get_info(uint8_t* addr, uint16_t* mtu);