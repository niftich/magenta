// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <bitmap/raw-bitmap.h>
#include <bitmap/storage.h>

#include <limits.h>
#include <stddef.h>

#include <magenta/types.h>
#include <mxtl/algorithm.h>
#include <mxtl/macros.h>

namespace {

const size_t kBits = sizeof(size_t) * 8;

// GetMask returns a 64-bit bitmask.  If the block of the bitmap we're looking
// at isn't the first or last, all bits are set.  Otherwise, the bits outside of
// [off,max) are cleared.  Bits are counted with the LSB as 0 and the MSB as 63.
//
// Examples:
//  GetMask(false, false, 16, 48) => 0xffffffffffffffff
//  GetMask(true,  false, 16, 48) => 0xffffffffffff0000
//  GetMask(false,  true, 16, 48) => 0x0000ffffffffffff
//  GetMask(true,   true, 16, 48) => 0x0000ffffffff0000
size_t GetMask(bool first, bool last, size_t off, size_t max) {
    size_t ones = 0;
    ones = ~ones;
    size_t mask = ones;
    if (first) {
        mask &= ones << (off % kBits);
    }
    if (last) {
        mask &= ones >> ((kBits - (max % kBits)) % kBits);
    }
    return mask;
}

// Translates a bit offset into a starting index in the bitmap array.
constexpr size_t FirstIdx(size_t bitoff) {
    return bitoff / kBits;
}

// Translates a max bit into a final index in the bitmap array.
constexpr size_t LastIdx(size_t bitmax) {
    return (bitmax - 1) / kBits;
}

// Counts the number of zeros.  It assumes everything in the array up to
// bits_[idx] is zero.
#if (SIZE_MAX == UINT_MAX)
#define CTZ(x) (x == 0 ? kBits : __builtin_ctz(x))
#elif (SIZE_MAX == ULONG_MAX)
#define CTZ(x) (x == 0 ? kBits : __builtin_ctzl(x))
#elif (SIZE_MAX == ULLONG_MAX)
#define CTZ(x) (x == 0 ? kBits : __builtin_ctzll(x))
#else
#error "Unsupported size_t length"
#endif
size_t CountZeros(size_t idx, size_t value) {
    return idx * kBits + CTZ(value);
}
#undef CTZ

} // namespace

namespace bitmap {

template <typename Storage>
RawBitmapGeneric<Storage>::RawBitmapGeneric() : size_(0), data_(nullptr) {}

// Resets the bitmap; clearing and resizing it.
template <typename Storage>
mx_status_t RawBitmapGeneric<Storage>::Reset(size_t size) {
    size_ = size;
    if (size_ == 0) {
        data_ = nullptr;
        return NO_ERROR;
    }
    size_t last_idx = LastIdx(size);
    mx_status_t status = bits_.Allocate(sizeof(size_t) * (last_idx + 1));
    if (status != NO_ERROR) {
        return status;
    }
    data_ = static_cast<size_t*>(bits_.GetData());
    ClearAll();
    return NO_ERROR;
}

template <typename Storage>
mx_status_t RawBitmapGeneric<Storage>::Shrink(size_t size) {
    if (size > size_) {
        return ERR_NO_MEMORY;
    }
    size_ = size;
    return NO_ERROR;
}

template <typename Storage>
size_t RawBitmapGeneric<Storage>::Scan(size_t bitoff, size_t bitmax, bool is_set) const {
    bitmax = mxtl::min(bitmax, size_);
    if (bitoff >= bitmax) {
        return bitmax;
    }
    size_t first_idx = FirstIdx(bitoff);
    size_t last_idx = LastIdx(bitmax);
    size_t i = first_idx;
    size_t value = 0;
    for (i = first_idx; i <= last_idx; ++i) {
        value = GetMask(i == first_idx, i == last_idx, bitoff, bitmax);
        if (is_set) {
            // If is_set=true, invert the mask, OR it with the value, and invert
            // it again to hopefully get all zeros.
            value = ~(~value | data_[i]);
        } else {
            // If is_set=false, just AND the mask with the value to hopefully
            // get all zeros.
            value &= data_[i];
        }
        if (value != 0) {
            break;
        }
    }
    return mxtl::min(bitmax, CountZeros(i, value));
}

template <typename Storage>
mx_status_t RawBitmapGeneric<Storage>::Find(bool is_set, size_t bitoff, size_t bitmax,
                                                size_t run_len, size_t* out) const {
    if (!out || bitmax <= bitoff) {
        return ERR_INVALID_ARGS;
    }
    size_t start = bitoff;
    while (bitoff - start < run_len && bitoff < bitmax) {
        start = Scan(bitoff, bitmax, !is_set);
        if (bitmax - start < run_len) {
            *out = bitmax;
            return ERR_NO_RESOURCES;
        }
        bitoff = Scan(start, start + run_len, is_set);
    }
    *out = start;
    return NO_ERROR;
}

template <typename Storage>
bool RawBitmapGeneric<Storage>::Get(size_t bitoff, size_t bitmax, size_t* first) const {
    bitmax = mxtl::min(bitmax, size_);
    size_t result = Scan(bitoff, bitmax, true);
    if (first) {
        *first = result;
    }
    return result == bitmax;
}

template <typename Storage>
mx_status_t RawBitmapGeneric<Storage>::Set(size_t bitoff, size_t bitmax) {
    if (bitoff > bitmax || bitmax > size_) {
        return ERR_INVALID_ARGS;
    }
    if (bitoff == bitmax) {
        return NO_ERROR;
    }
    size_t first_idx = FirstIdx(bitoff);
    size_t last_idx = LastIdx(bitmax);
    for (size_t i = first_idx; i <= last_idx; ++i) {
        data_[i] |=
                GetMask(i == first_idx, i == last_idx, bitoff, bitmax);
    }
    return NO_ERROR;
}

template <typename Storage>
mx_status_t RawBitmapGeneric<Storage>::Clear(size_t bitoff, size_t bitmax) {
    if (bitoff > bitmax || bitmax > size_) {
        return ERR_INVALID_ARGS;
    }
    if (bitoff == bitmax) {
        return NO_ERROR;
    }
    size_t first_idx = FirstIdx(bitoff);
    size_t last_idx = LastIdx(bitmax);
    for (size_t i = first_idx; i <= last_idx; ++i) {
        data_[i] &=
                ~(GetMask(i == first_idx, i == last_idx, bitoff, bitmax));
    }
    return NO_ERROR;
}

template <typename Storage>
void RawBitmapGeneric<Storage>::ClearAll() {
    if (size_ == 0) {
        return;
    }
    size_t last_idx = LastIdx(size_);
    for (size_t i = 0; i <= last_idx; ++i) {
        data_[i] = 0;
    }
}

#ifdef __Fuchsia__
template class RawBitmapGeneric<VmoStorage>;
#endif
template class RawBitmapGeneric<DefaultStorage>;

} // namespace bitmap
