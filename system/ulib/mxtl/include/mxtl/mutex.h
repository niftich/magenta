// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#ifdef __cplusplus

#include <magenta/compiler.h>
#include <mxtl/macros.h>

// Notes about class Mutex
//
// Mutex is a C++ helper class intended to wrap a mutex-style synchronization
// primative and provide a common interface for library code which is intended
// to be shared between user-mode and kernel code.  It is also responsible for
// automatically initializing and destroying the internal mutex object.
//
// For user-mode code, Mutex is defined in the mxtl namespace.  For kernel mode
// code, to maintain compatibility with existing code, Mutex is introduced into
// the global namespace, and a using alias is introduced in the mxtl namespace.
// The implication of this is that shared code should always use mxtl::Mutex
// instead of the global Mutex
#if _KERNEL
#include <kernel/mutex.h>
#include <sys/types.h>

class __TA_CAPABILITY("mutex") Mutex {
public:
    constexpr Mutex() : mutex_(MUTEX_INITIAL_VALUE(mutex_)) { }
    ~Mutex() { mutex_destroy(&mutex_); }
    void Acquire() __TA_ACQUIRE() { mutex_acquire(&mutex_); }
    void Release() __TA_RELEASE() { mutex_release(&mutex_); }

    bool IsHeld() const {
        return is_mutex_held(&mutex_);
    }

    mutex_t* GetInternal() __TA_RETURN_CAPABILITY(mutex_) {
        return &mutex_;
    }

    // suppress default constructors
    DISALLOW_COPY_ASSIGN_AND_MOVE(Mutex);

private:
    mutex_t mutex_;
};

namespace mxtl { using Mutex = ::Mutex; }

#else   // if _KERNEL

#include <magenta/types.h>
#include <threads.h>

namespace mxtl {

class __TA_CAPABILITY("mutex") Mutex {
public:
#ifdef MTX_INIT
    constexpr Mutex() : mutex_(MTX_INIT) { }
#else
    Mutex() { mtx_init(&mutex_, mtx_plain); }
#endif
    ~Mutex() { mtx_destroy(&mutex_); }
    void Acquire() __TA_ACQUIRE() { mtx_lock(&mutex_); }
    void Release() __TA_RELEASE() { mtx_unlock(&mutex_); }

    /* IsHeld is not supported by the Mutex wrapper in user-mode as C11 mtx_t
     * instances do not support a direct IsHeld style check.  A possible
     * implementation could be built out of mtx_trylock, but would require
     * either relaxing away the const constraint on the method signature, or
     * flagging the mutex_ member as mutable */

    mtx_t* GetInternal() __TA_RETURN_CAPABILITY(mutex_) {
        return &mutex_;
    }

    // suppress default constructors
    DISALLOW_COPY_ASSIGN_AND_MOVE(Mutex);

private:
    mtx_t mutex_;
};

}

#endif  // if _KERNEL
#endif  // ifdef __cplusplus
