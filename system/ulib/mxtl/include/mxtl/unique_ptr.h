// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <stdlib.h>
#include <mxtl/macros.h>
#include <mxtl/recycler.h>
#include <mxtl/type_support.h>

namespace mxtl {

// This is a simplified version of std::unique_ptr<> that can automatically
// dispose a pointer when it goes out of scope.
//
// Compared to std::unique_ptr, it does not support custom deleters and has
// restrictive type conversion semantics.
template <typename T>
class unique_ptr {
public:
    constexpr unique_ptr() : ptr_(nullptr) {}
    constexpr unique_ptr(decltype(nullptr)) : unique_ptr() {}

    explicit unique_ptr(T* t) : ptr_(t) { }

    ~unique_ptr() {
        recycle(ptr_);
    }

    unique_ptr(unique_ptr&& o) : ptr_(o.release()) {}
    unique_ptr& operator=(unique_ptr&& o) {
        reset(o.release());
        return *this;
    }

    unique_ptr& operator=(decltype(nullptr)) {
        reset();
        return *this;
    }

    // Comparison against nullptr operators (of the form, myptr == nullptr).
    bool operator==(decltype(nullptr)) const { return (ptr_ == nullptr); }
    bool operator!=(decltype(nullptr)) const { return (ptr_ != nullptr); }

    // Comparison against other unique_ptr<>'s.
    bool operator==(const unique_ptr& o) const { return ptr_ == o.ptr_; }
    bool operator!=(const unique_ptr& o) const { return ptr_ != o.ptr_; }
    bool operator< (const unique_ptr& o) const { return ptr_ <  o.ptr_; }
    bool operator<=(const unique_ptr& o) const { return ptr_ <= o.ptr_; }
    bool operator> (const unique_ptr& o) const { return ptr_ >  o.ptr_; }
    bool operator>=(const unique_ptr& o) const { return ptr_ >= o.ptr_; }

    // move semantics only
    DISALLOW_COPY_AND_ASSIGN_ALLOW_MOVE(unique_ptr);

    T* release() {
        T* t = ptr_;
        ptr_ = nullptr;
        return t;
    }
    void reset(T* t = nullptr) {
        recycle(ptr_);
        ptr_ = t;
    }
    void swap(unique_ptr& other) {
        T* t = ptr_;
        ptr_ = other.ptr_;
        other.ptr_ = t;
    }

    T* get() const {
        return ptr_;
    }

    explicit operator bool() const {
        return static_cast<bool>(ptr_);
    }

    T& operator*() const {
        return *ptr_;
    }
    T* operator->() const {
        return ptr_;
    }

    // Implicit upcasting via construction.
    //
    // We permit implicit casting of a unique_ptr<U> to a unique_ptr<T> as long
    // as...
    //
    // 1) U* is implicitly convertible to a T*
    // 2) Neither T nor U are a class/struct type, or both T and U are
    //    class/struct types, and T has a virtual destructor.
    //
    // Note: we do this via an implicit converting constructor (instead of a
    // user-defined conversion operator) so that we can demand that we are
    // converting from a properly moved rvalue reference.
    //
    // Also Note:  This behavior is *not* the same as std::unique_ptr.
    // std::unique_ptr only cares about point #1.  Its behavior emulates raw
    // pointers and will gladly let you implicitly convert a class U to a class
    // T as an implicit upcast regardless of whether or not T has a virtual
    // destructor.
    template <typename U,
              typename = typename enable_if<is_convertible_pointer<U*, T*>::value>::type>
    unique_ptr(unique_ptr<U>&& o) : ptr_(o.release()) {
        static_assert((is_class<T>::value == is_class<U>::value) &&
                     (!is_class<T>::value || has_virtual_destructor<T>::value),
                "Cannot convert unique_ptr<U> to unique_ptr<T> unless neither T "
                "nor U are class/struct types, or T has a virtual destructor");
    }

private:
    static void recycle(T* ptr) {
        enum { type_must_be_complete = sizeof(T) };
        if (ptr) {
            if (::mxtl::internal::has_mxtl_recycle<T>::value) {
                ::mxtl::internal::recycler<T>::recycle(ptr);
            } else {
                delete ptr;
            }
        }
    }

    T* ptr_;
};

template <typename T>
class unique_ptr<T[]> {
public:
    constexpr unique_ptr() : ptr_(nullptr) {}
    constexpr unique_ptr(decltype(nullptr)) : unique_ptr() {}

    explicit unique_ptr(T* array) : ptr_(array) {}

    unique_ptr(unique_ptr&& other) : ptr_(other.release()) {}

    ~unique_ptr() {
        enum { type_must_be_complete = sizeof(T) };
        if (ptr_) delete[] ptr_;
    }

    unique_ptr& operator=(unique_ptr&& o) {
        reset(o.release());
        return *this;
    }

    // Comparison against nullptr operators (of the form, myptr == nullptr).
    bool operator==(decltype(nullptr)) const { return (ptr_ == nullptr); }
    bool operator!=(decltype(nullptr)) const { return (ptr_ != nullptr); }

    // Comparison against other unique_ptr<>'s.
    bool operator==(const unique_ptr& o) const { return ptr_ == o.ptr_; }
    bool operator!=(const unique_ptr& o) const { return ptr_ != o.ptr_; }
    bool operator< (const unique_ptr& o) const { return ptr_ <  o.ptr_; }
    bool operator<=(const unique_ptr& o) const { return ptr_ <= o.ptr_; }
    bool operator> (const unique_ptr& o) const { return ptr_ >  o.ptr_; }
    bool operator>=(const unique_ptr& o) const { return ptr_ >= o.ptr_; }

    unique_ptr(const unique_ptr& o) = delete;
    unique_ptr& operator=(const unique_ptr& o) = delete;

    T* release() {
        T* t = ptr_;
        ptr_ = nullptr;
        return t;
    }
    void reset(T* t = nullptr) {
        enum { type_must_be_complete = sizeof(T) };
        if (ptr_) delete[] ptr_;
        ptr_ = t;
    }
    void swap(unique_ptr& other) {
        T* t = ptr_;
        ptr_ = other.ptr_;
        other.ptr_ = t;
    }

    T* get() const {
        return ptr_;
    }

    explicit operator bool() const {
        return static_cast<bool>(ptr_);
    }
    T& operator[](size_t i) const {
        return ptr_[i];
    }

private:
    T* ptr_;
};

// Comparison against nullptr operators (of the form, nullptr == myptr) for T and T[]
template <typename T>
static inline bool operator==(decltype(nullptr), const unique_ptr<T>& ptr) {
    return (ptr.get() == nullptr);
}

template <typename T>
static inline bool operator!=(decltype(nullptr), const unique_ptr<T>& ptr) {
    return (ptr.get() != nullptr);
}

}  // namespace mxtl
