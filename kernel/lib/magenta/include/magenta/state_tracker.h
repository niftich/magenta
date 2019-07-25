// Copyright 2016 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#pragma once

#include <stdint.h>

#include <kernel/mutex.h>
#include <kernel/spinlock.h>
#include <magenta/state_observer.h>
#include <magenta/types.h>
#include <mxtl/canary.h>
#include <mxtl/intrusive_double_list.h>

class Handle;

class CookieJar {
public:
    CookieJar() : scope_(MX_KOID_INVALID), cookie_(0) {}
    mx_koid_t scope_;
    uint64_t cookie_;
};

class StateTracker {
public:
    StateTracker(mx_signals_t signals = 0u) : signals_(signals | MX_SIGNAL_LAST_HANDLE) { }

    StateTracker(const StateTracker& o) = delete;
    StateTracker& operator=(const StateTracker& o) = delete;

    // Add an observer.
    void AddObserver(StateObserver* observer, const StateObserver::CountInfo* cinfo);

    // Remove an observer (which must have been added).
    void RemoveObserver(StateObserver* observer);

    // Called when observers of the handle's state (e.g., waits on the handle) should be
    // "cancelled", i.e., when a handle (for the object that owns this StateTracker) is being
    // destroyed or transferred.
    void Cancel(Handle* handle);

    // Like Cancel() but issued via via mx_port_cancel().
    void CancelByKey(Handle* handle, const void* port, uint64_t key);

    // Notify others of a change in state (possibly waking them). (Clearing satisfied signals or
    // setting satisfiable signals should not wake anyone.)
    void UpdateState(mx_signals_t clear_mask, mx_signals_t set_mask);

    // Notify others of a change in state (possibly waking them) in an edge-triggered
    // manner.  Waiters on strobe_mask will wake, but the tracked state is unmodified.
    void StrobeState(mx_signals_t strobe_mask);

    // Nofity others with MX_SIGNAL_LAST_HANDLE if the value pointed by |count| is 1. This
    // value is allowed to mutate by other threads while this call is executing.
    void UpdateLastHandleSignal(uint32_t* count);

    mx_signals_t GetSignalsState() { return signals_; }

    using ObserverList = mxtl::DoublyLinkedList<StateObserver*, StateObserverListTraits>;

    // Accessors for CookieJars
    // These live with the state tracker so they can make use of the state tracker's
    // lock (since not all objects have their own locks, but all Dispatchers that are
    // cookie-capable have state trackers)
    mx_status_t SetCookie(CookieJar* cookiejar, mx_koid_t scope, uint64_t cookie);
    mx_status_t GetCookie(CookieJar* cookiejar, mx_koid_t scope, uint64_t* cookie);
    mx_status_t InvalidateCookie(CookieJar *cookiejar);

private:
    // Returns true if one of the observers have been signaled. False otherwise.
    bool UpdateInternalLocked(ObserverList* obs_to_remove, mx_signals_t signals) TA_REQ(lock_);

    mxtl::Canary<mxtl::magic("STRK")> canary_;

    mx_signals_t signals_;
    Mutex lock_;

    // Active observers are elements in |observers_|.
    ObserverList observers_ TA_GUARDED(lock_);
};
