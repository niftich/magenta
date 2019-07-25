// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// TODO(dje): Revisit where things should live once things settle.

#pragma once

#include <magenta/compiler.h>
#include <magenta/device/ioctl.h>
#include <magenta/device/ioctl-wrapper.h>
#include <magenta/types.h>
#include <stddef.h>

__BEGIN_CDECLS

#if !defined(__x86_64__)
#error "unsupported architecture"
#endif

// Bits in the IA32_RTIT_CTL MSR.
// These bits are writable by the user with ioctl_ipt_set_ctl_config.
// The driver will override a setting if it's unsafe (e.g., causes #gpf).
// TODO(dje): Append _MASK, add _BIT, _LEN.
#define IPT_CTL_CYC_EN (1ULL << 1)
#define IPT_CTL_OS_ALLOWED (1ULL << 2)
#define IPT_CTL_USER_ALLOWED (1ULL << 3)
#define IPT_CTL_POWER_EVENT_EN (1ULL << 4)
#define IPT_CTL_FUP_ON_PTW (1ULL << 5)
#define IPT_CTL_CR3_FILTER (1ULL << 7)
#define IPT_CTL_MTC_EN (1ULL << 9)
#define IPT_CTL_TSC_EN (1ULL << 10)
#define IPT_CTL_DIS_RETC (1ULL << 11)
#define IPT_CTL_PTW_EN (1ULL << 12)
#define IPT_CTL_BRANCH_EN (1ULL << 13)
#define IPT_CTL_MTC_FREQ (0xfULL << 14)
#define IPT_CTL_CYC_THRESH (0xfULL << 19)
#define IPT_CTL_PSB_FREQ (0xfULL << 24)
#define IPT_CTL_ADDR0 (0xfULL << 32)
#define IPT_CTL_ADDR1 (0xfULL << 36)
#define IPT_CTL_ADDR2 (0xfULL << 40)
#define IPT_CTL_ADDR3 (0xfULL << 44)

// Other bits in IA32_RTIT_CTL MSR, not writable with ioctl_ipt_set_ctl_config.
#define IPT_CTL_TRACE_EN (1ULL<<0)
#define IPT_CTL_FABRIC_EN (1ULL<<6)
#define IPT_CTL_TOPA (1ULL<<8)

// Masks for reading IA32_RTIT_STATUS.
#define IPT_STATUS_FILTER_EN (1ULL<<0)
#define IPT_STATUS_CONTEXT_EN (1ULL<<1)
#define IPT_STATUS_TRIGGER_EN (1ULL<<2)
#define IPT_STATUS_ERROR (1ULL<<4)
#define IPT_STATUS_STOPPED (1ULL<<5)

// Maximum number of address ranges that h/w may support.
#define IPT_MAX_NUM_ADDR_RANGES 4

// Valid ToPA entry sizes.
#define IPT_TOPA_MIN_SHIFT 12
#define IPT_TOPA_MAX_SHIFT 27

// These bits are for internal use

// Macros for building entries for the Table of Physical Addresses
#define IPT_TOPA_ENTRY_PHYS_ADDR(x) ((uint64_t)(x) & ~((1ULL<<12)-1))
#define IPT_TOPA_ENTRY_SIZE(size_log2) ((uint64_t)((size_log2) - 12) << 6)
#define IPT_TOPA_ENTRY_STOP (1ULL << 4)
#define IPT_TOPA_ENTRY_INT (1ULL << 2)
#define IPT_TOPA_ENTRY_END (1ULL << 0)

// Macros for extracting info from ToPA entries
#define IPT_TOPA_ENTRY_EXTRACT_PHYS_ADDR(e) \
  ((mx_paddr_t)((e) & ~((1ULL<<12)-1)))
#define IPT_TOPA_ENTRY_EXTRACT_SIZE(e) ((uint)((((e) >> 6) & 0xf) + 12))

// Arbitarily picked constants for ourselves.
// ToPA tables are 16KB in size (technically can be up to 256MB).
// A 16KB table provides 2047 non-END entries, so at the
// minimum can provide a capture buffer of just under 8MB.
#define IPT_TOPA_MAX_TABLE_ENTRIES 2048

// The PT register set.
// This is accessed via mtrace, but basically it's a regset.
typedef struct {
    uint64_t ctl;
    uint64_t status;
    uint64_t output_base;
    uint64_t output_mask_ptrs;
    uint64_t cr3_match;
    struct {
        uint64_t a,b;
    } addr_ranges[IPT_MAX_NUM_ADDR_RANGES];
} mx_x86_pt_regs_t;

// Two "modes" of tracing are supported:
// trace each cpu, regardless of what's running on it
#define IPT_MODE_CPUS 0
// trace specific threads
#define IPT_MODE_THREADS 1

///////////////////////////////////////////////////////////////////////////////

// ioctls

// set the trace mode, either cpus or threads
// Input: one of IPT_MODE_CPUS, IPT_MODE_THREADS
#define IOCTL_IPT_SET_MODE \
    IOCTL(IOCTL_KIND_DEFAULT, IOCTL_FAMILY_IPT, 0)
IOCTL_WRAPPER_IN(ioctl_ipt_set_mode, IOCTL_IPT_SET_MODE, uint32_t);

typedef struct {
    uint32_t num_buffers;
    // #pages as a power of 2
    uint32_t buffer_order;
    bool is_circular;
    uint64_t ctl;
    uint64_t cr3_match;
    struct {
        uint64_t a,b;
    } addr_ranges[IPT_MAX_NUM_ADDR_RANGES];
} ioctl_ipt_buffer_config_t;

// allocate a trace buffer
// Input: ioctl_ipt_buffer_config_t
// Output: trace buffer descriptor (think file descriptor for trace buffers)
// When tracing cpus, buffers are auto-assigned to cpus: the resulting trace
// buffer descriptor is the number of the cpu using the buffer.
#define IOCTL_IPT_ALLOC_BUFFER \
    IOCTL(IOCTL_KIND_DEFAULT, IOCTL_FAMILY_IPT, 1)
IOCTL_WRAPPER_INOUT(ioctl_ipt_alloc_buffer, IOCTL_IPT_ALLOC_BUFFER, ioctl_ipt_buffer_config_t, uint32_t);

typedef struct {
    // for IOCTL_KIND_SET_HANDLE first element must be the handle
    mx_handle_t thread;
    uint32_t descriptor;
} ioctl_ipt_assign_buffer_thread_t;

// assign a buffer to a thread
// Input: ioctl_ipt_assign_buffer_thread_t
#define IOCTL_IPT_ASSIGN_BUFFER_THREAD \
    IOCTL(IOCTL_KIND_SET_HANDLE, IOCTL_FAMILY_IPT, 3)
IOCTL_WRAPPER_IN(ioctl_ipt_assign_buffer_thread, IOCTL_IPT_ASSIGN_BUFFER_THREAD,
                 ioctl_ipt_assign_buffer_thread_t);

// release a buffer from a thread
// Input: ioctl_ipt_assign_buffer_thread_t
#define IOCTL_IPT_RELEASE_BUFFER_THREAD \
    IOCTL(IOCTL_KIND_SET_HANDLE, IOCTL_FAMILY_IPT, 5)
IOCTL_WRAPPER_IN(ioctl_ipt_release_buffer_thread, IOCTL_IPT_RELEASE_BUFFER_THREAD,
                 ioctl_ipt_assign_buffer_thread_t);

// return config data for a trace buffer
// Input: trace buffer descriptor
// Output: ioctl_ipt_trace_buffer_config_t
#define IOCTL_IPT_GET_BUFFER_CONFIG \
    IOCTL(IOCTL_KIND_DEFAULT, IOCTL_FAMILY_IPT, 6)
IOCTL_WRAPPER_INOUT(ioctl_ipt_get_buffer_config, IOCTL_IPT_GET_BUFFER_CONFIG,
                    uint32_t, ioctl_ipt_buffer_config_t);

// This contains the run-time produced data about the buffer.
// Not the trace data itself, just info about the data.
typedef struct {
    // N.B. This is the offset in the buffer where tracing stopped (treating
    // all buffers as one large one). If using a circular buffer then all of
    // the buffer may contain data, there's no current way to know if tracing
    // wrapped.
    uint64_t capture_end;
} ioctl_ipt_buffer_info_t;

// get trace data associated with the buffer
// Input: trace buffer descriptor
// Output: ioctl_ipt_buffer_info_t
#define IOCTL_IPT_GET_BUFFER_INFO \
    IOCTL(IOCTL_KIND_DEFAULT, IOCTL_FAMILY_IPT, 7)
IOCTL_WRAPPER_INOUT(ioctl_ipt_get_buffer_info, IOCTL_IPT_GET_BUFFER_INFO,
                    uint32_t, ioctl_ipt_buffer_info_t);

typedef struct {
    uint32_t descriptor;
    uint32_t buffer_num;
} ioctl_ipt_buffer_handle_req_t;

// return a handle of a trace buffer
// There is no API to get N handles, we have to get them one at a time.
// [There's no point in trying to micro-optimize this and, say, get 3 at
// a time.]
#define IOCTL_IPT_GET_BUFFER_HANDLE \
    IOCTL(IOCTL_KIND_GET_HANDLE, IOCTL_FAMILY_IPT, 8)
IOCTL_WRAPPER_INOUT(ioctl_ipt_get_buffer_handle, IOCTL_IPT_GET_BUFFER_HANDLE,
                    ioctl_ipt_buffer_handle_req_t, mx_handle_t);

// free a trace buffer
// Input: trace buffer descriptor
#define IOCTL_IPT_FREE_BUFFER \
    IOCTL(IOCTL_KIND_DEFAULT, IOCTL_FAMILY_IPT, 9)
IOCTL_WRAPPER_IN(ioctl_ipt_free_buffer, IOCTL_IPT_FREE_BUFFER,
                 uint32_t);

// must be called prior to START, allocate buffers of the specified size
#define IOCTL_IPT_CPU_MODE_ALLOC \
    IOCTL(IOCTL_KIND_DEFAULT, IOCTL_FAMILY_IPT, 10)
IOCTL_WRAPPER(ioctl_ipt_cpu_mode_alloc, IOCTL_IPT_CPU_MODE_ALLOC);

// turn on processor tracing
#define IOCTL_IPT_CPU_MODE_START \
    IOCTL(IOCTL_KIND_DEFAULT, IOCTL_FAMILY_IPT, 11)
IOCTL_WRAPPER(ioctl_ipt_cpu_mode_start, IOCTL_IPT_CPU_MODE_START);

// turn off processor tracing
#define IOCTL_IPT_CPU_MODE_STOP \
    IOCTL(IOCTL_KIND_DEFAULT, IOCTL_FAMILY_IPT, 12)
IOCTL_WRAPPER(ioctl_ipt_cpu_mode_stop, IOCTL_IPT_CPU_MODE_STOP);

// release resources allocated with IOCTL_IPT_CPU_MODE_ALLOC
// must be called prior to reconfiguring buffer sizes
#define IOCTL_IPT_CPU_MODE_FREE \
    IOCTL(IOCTL_KIND_DEFAULT, IOCTL_FAMILY_IPT, 13)
IOCTL_WRAPPER(ioctl_ipt_cpu_mode_free, IOCTL_IPT_CPU_MODE_FREE);

__END_CDECLS
