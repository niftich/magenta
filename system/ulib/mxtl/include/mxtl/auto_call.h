// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <mxtl/macros.h>
#include <mxtl/type_support.h>

// RAII class to automatically call a function-like thing as it goes out of
// scope
//
// Examples:
//
//    extern int foo();
//    int a;
//
//    auto ac = mxtl::MakeAutoCall([&](){ a = 1; });
//    auto ac2 = mxtl::MakeAutoCall(foo);
//
//    auto func = [&](){ a = 2; };
//    mxtl::AutoCall<decltype(func)> ac3(func);
//    mxtl::AutoCall<decltype(&foo)> ac4(&foo);
//
//    // abort the call
//    ac2.cancel();
namespace mxtl {

template <typename T>
class AutoCall {
public:
    constexpr explicit AutoCall(T c) : call_(move(c)) {}
    ~AutoCall() {
        call();
    }

    // move semantics
    AutoCall(AutoCall&& c) : call_(move(c.call_)), active_(c.active_) {
        c.cancel();
    }

    AutoCall& operator=(AutoCall&& c) {
        call_ = move(c.call_);
        c.cancel();
    }

    // no copy
    DISALLOW_COPY_AND_ASSIGN_ALLOW_MOVE(AutoCall);

    // cancel the eventual call
    void cancel() {
        active_ = false;
    }

    // call it immediately
    void call() {
        bool active = active_;
        cancel();
        if (active) (call_)();
    }

private:
    T call_;
    bool active_ = true;
};

// helper routine to create an autocall object without needing template
// specialization
template <typename T>
inline AutoCall<T> MakeAutoCall(T c) {
    return AutoCall<T>(move(c));
}

};  // namespace mxtl
