// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

// setup networking
int netifc_open(void);

// process inbound packet(s)
void netifc_poll(void);

// return nonzero if interface exists
int netifc_active(void);

// shut down networking
void netifc_close(void);

// set a timer to expire after ms milliseconds
void netifc_set_timer(uint32_t ms);

// returns true once the timer has expired
int netifc_timer_expired(void);
