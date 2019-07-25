// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#ifdef __cplusplus
#include <magenta/compiler.h>

// Notes about class NullLock
//
// NullLock is a stub class which exposes the same API as mxtl::Mutex, but does
// nothing (it neither allocates nor locks).  It may be used with mxtl templated
// utility classes whose locking behavior is determined by passing a lock class
// type as a template parameter to get a no-locking behavior.
namespace mxtl {

class __TA_CAPABILITY("mutex") NullLock {
public:
    constexpr NullLock() { }
    void Acquire() __TA_ACQUIRE() { }
    void Release() __TA_RELEASE() { }
};

}  // namespace mxtl
#endif  // ifdef __cplusplus
