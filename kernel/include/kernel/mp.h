// Copyright 2016 The Fuchsia Authors
// Copyright (c) 2014 Travis Geiselbrecht
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#pragma once

#include <magenta/compiler.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <kernel/mutex.h>
#include <kernel/thread.h>

__BEGIN_CDECLS;

typedef uint32_t mp_cpu_mask_t;
typedef void (*mp_ipi_task_func_t)(void *context);
typedef void (*mp_sync_task_t)(void *context);

#define MP_CPU_ALL_BUT_LOCAL (UINT32_MAX)
#define MP_CPU_ALL (1U<<31)
static_assert(SMP_MAX_CPUS <= 31, "");

/* by default, mp_mbx_reschedule does not signal to cpus that are running realtime
 * threads. Override this behavior.
 */
#define MP_RESCHEDULE_FLAG_REALTIME (0x1)

typedef enum {
    MP_IPI_GENERIC,
    MP_IPI_RESCHEDULE,
    MP_IPI_HALT,
} mp_ipi_t;

#define MAX_IPI (3)

#ifdef WITH_SMP
void mp_init(void);

void mp_reschedule(mp_cpu_mask_t target, uint flags);
void mp_sync_exec(mp_cpu_mask_t target, mp_sync_task_t task, void *context);
void mp_set_curr_cpu_online(bool online);
void mp_set_curr_cpu_active(bool active);

status_t mp_hotplug_cpu(uint cpu_id);
status_t mp_unplug_cpu(uint cpu_id);

/* called from arch code during reschedule irq */
enum handler_return mp_mbx_reschedule_irq(void);
/* called from arch code during generic task irq */
enum handler_return mp_mbx_generic_irq(void);

/* represents a pending task for some number of CPUs to execute */
struct mp_ipi_task {
    struct list_node node;

    mp_ipi_task_func_t func;
    void *context;
};

/* global mp state to track what the cpus are up to */
struct mp_state {
    /* cpus that are currently online */
    volatile mp_cpu_mask_t online_cpus;
    /* cpus that are currently schedulable */
    volatile mp_cpu_mask_t active_cpus;

    /* only safely accessible with thread lock held */
    mp_cpu_mask_t idle_cpus;
    mp_cpu_mask_t realtime_cpus;

    spin_lock_t ipi_task_lock;
    /* list of outstanding tasks for CPUs to execute.  Should only be
     * accessed with the ipi_task_lock held */
    struct list_node ipi_task_list[SMP_MAX_CPUS];

    /* lock for serializing CPU hotplug/unplug operations */
    mutex_t hotplug_lock;
};

extern struct mp_state mp;

static inline int mp_is_cpu_active(uint cpu)
{
    return atomic_load((int *)&mp.active_cpus) & (1 << cpu);
}

static inline int mp_is_cpu_idle(uint cpu)
{
    return mp.idle_cpus & (1 << cpu);
}

static inline int mp_is_cpu_online(uint cpu)
{
    return mp.online_cpus & (1 << cpu);
}

/* must be called with the thread lock held */
static inline void mp_set_cpu_idle(uint cpu)
{
    mp.idle_cpus |= 1U << cpu;
}

static inline void mp_set_cpu_busy(uint cpu)
{
    mp.idle_cpus &= ~(1U << cpu);
}

static inline mp_cpu_mask_t mp_get_idle_mask(void)
{
    return mp.idle_cpus;
}

static inline mp_cpu_mask_t mp_get_active_mask(void)
{
    return atomic_load((int *)&mp.active_cpus);
}

static inline mp_cpu_mask_t mp_get_online_mask(void)
{
    return mp.online_cpus;
}

static inline void mp_set_cpu_realtime(uint cpu)
{
    mp.realtime_cpus |= 1U << cpu;
}

static inline void mp_set_cpu_non_realtime(uint cpu)
{
    mp.realtime_cpus &= ~(1U << cpu);
}

static inline mp_cpu_mask_t mp_get_realtime_mask(void)
{
    return mp.realtime_cpus;
}
#else
static inline void mp_init(void) {}
static inline void mp_reschedule(mp_cpu_mask_t target, uint flags) {}
static inline void mp_sync_exec(mp_cpu_mask_t target, mp_sync_task_t task, void *context)
{
    if (target != MP_CPU_ALL &&
        (!(target & 0x1) || target == MP_CPU_ALL_BUT_LOCAL)) return;
    spin_lock_saved_state_t irqstate;
    arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_INTERRUPTS);
    task(context);
    arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_INTERRUPTS);
}
static inline void mp_set_curr_cpu_active(bool active) {}
static inline void mp_set_curr_cpu_online(bool online) {}

static inline enum handler_return mp_mbx_reschedule_irq(void) { return INT_NO_RESCHEDULE; }
static inline enum handler_return mp_mbx_generic_irq(void) { return INT_NO_RESCHEDULE; }

// only one cpu exists in UP and if you're calling these functions, it's active...
static inline int mp_is_cpu_active(uint cpu) { return 1; }
static inline int mp_is_cpu_idle(uint cpu) { return (get_current_thread()->flags & THREAD_FLAG_IDLE) != 0; }
static inline int mp_is_cpu_online(uint cpu) { return 1; }

static inline void mp_set_cpu_idle(uint cpu) {}
static inline void mp_set_cpu_busy(uint cpu) {}

static inline mp_cpu_mask_t mp_get_idle_mask(void) { return 0; }

static inline void mp_set_cpu_realtime(uint cpu) {}
static inline void mp_set_cpu_non_realtime(uint cpu) {}

static inline mp_cpu_mask_t mp_get_realtime_mask(void) { return 0; }

static inline mp_cpu_mask_t mp_get_active_mask(void) { return 1; }
static inline mp_cpu_mask_t mp_get_online_mask(void) { return 1; }
#endif

__END_CDECLS;
