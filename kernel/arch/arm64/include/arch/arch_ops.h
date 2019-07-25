// Copyright 2016 The Fuchsia Authors
// Copyright (c) 2008-2014 Travis Geiselbrecht
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#pragma once

#ifndef ASSEMBLY

#include <stdbool.h>
#include <magenta/compiler.h>
#include <reg.h>
#include <arch/arm64.h>
#include <arch/arm64/mp.h>

__BEGIN_CDECLS

#define ENABLE_CYCLE_COUNTER 1

// override of some routines
static inline void arch_enable_ints(void)
{
    CF;
    __asm__ volatile("msr daifclr, #2" ::: "memory");
}

static inline void arch_disable_ints(void)
{
    __asm__ volatile("msr daifset, #2" ::: "memory");
    CF;
}

static inline bool arch_ints_disabled(void)
{
    unsigned long state;

    __asm__ volatile("mrs %0, daif" : "=r"(state));
    state &= (1<<7);

    return !!state;
}

static inline void arch_enable_fiqs(void)
{
    CF;
    __asm__ volatile("msr daifclr, #1" ::: "memory");
}

static inline void arch_disable_fiqs(void)
{
    __asm__ volatile("msr daifset, #1" ::: "memory");
    CF;
}

// XXX
static inline bool arch_fiqs_disabled(void)
{
    unsigned long state;

    __asm__ volatile("mrs %0, daif" : "=r"(state));
    state &= (1<<6);

    return !!state;
}

static inline void arch_spinloop_pause(void)
{
    __asm__ volatile("wfe" ::: "memory");
}

static inline void arch_spinloop_signal(void)
{
    __asm__ volatile("sev" ::: "memory");
}

#define mb()        __asm__ volatile("dsb sy" : : : "memory")
#define rmb()       __asm__ volatile("dsb ld" : : : "memory")
#define wmb()       __asm__ volatile("dsb st" : : : "memory")

#ifdef WITH_SMP
#define smp_mb()    __asm__ volatile("dmb ish" : : : "memory")
#define smp_rmb()   __asm__ volatile("dmb ishld" : : : "memory")
#define smp_wmb()   __asm__ volatile("dmb ishst" : : : "memory")
#else
#define smp_mb()    CF
#define smp_wmb()   CF
#define smp_rmb()   CF
#endif

static inline uint64_t arch_cycle_count(void)
{
    return ARM64_READ_SYSREG(pmccntr_el0);
}

static inline uint32_t arch_dcache_line_size(void) {
    // According to ARMv8 manual D7.2.21, the Cache Type Register (CTR)
    // is a 32 bit control register that contains the smallest icache
    // line size and smallest dcache line size.
    uint32_t ctr = (uint32_t)ARM64_READ_SYSREG(ctr_el0);

    // Bits 16:19 of the CTR tell us the log_2 of the number of _words_
    // in the smallest _data_ cache line on this system.
    uint32_t dcache_log2 = (ctr >> 16) & 0xf;
    return 4u << dcache_log2;
}

// Log architecture-specific data for process creation.
// This can only be called after the process has been created and before
// it is running: |aspace| is assumed to live across the call.
// Alas we can't use mx_koid_t here as the arch layer is at a lower level
// than magenta.
struct arch_aspace;
static inline void arch_trace_process_create(uint64_t pid, const struct arch_aspace* aspace) {
    // nothing to do
}

__END_CDECLS

#endif // ASSEMBLY
