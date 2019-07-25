// Copyright 2017 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#pragma once

#include <magenta/handle.h>
#include <mxtl/intrusive_double_list.h>

// Delete handles out-of-band, using a deferred procedure call.
void ReapHandles(mxtl::DoublyLinkedList<Handle*>* handles);
void ReapHandles(Handle** handles, uint32_t num_handles);
