// Copyright 2016 The Fuchsia Authors
// Copyright (c) 2008-2014 Travis Geiselbrecht
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef __KERNEL_EVENT_H
#define __KERNEL_EVENT_H

#include <magenta/compiler.h>
#include <stdbool.h>
#include <sys/types.h>
#include <kernel/thread.h>

__BEGIN_CDECLS;

#define EVENT_MAGIC (0x65766E74)  // "evnt"

typedef struct event {
    int magic;
    bool signaled;
    uint flags;
    wait_queue_t wait;
} event_t;

#define EVENT_FLAG_AUTOUNSIGNAL 1

#define EVENT_INITIAL_VALUE(e, initial, _flags) \
{ \
    .magic = EVENT_MAGIC, \
    .signaled = initial, \
    .flags = _flags, \
    .wait = WAIT_QUEUE_INITIAL_VALUE((e).wait), \
}

/* Rules for Events:
 * - Events may be signaled from interrupt context *but* the reschedule
 *   parameter must be false in that case.
 * - Events may not be waited upon from interrupt context.
 * - Events without FLAG_AUTOUNSIGNAL:
 *   - Wake up any waiting threads when signaled.
 *   - Continue to do so (no threads will wait) until unsignaled.
 * - Events with FLAG_AUTOUNSIGNAL:
 *   - If one or more threads are waiting when signaled, one thread will
 *     be woken up and return.  The signaled state will not be set.
 *   - If no threads are waiting when signaled, the Event will remain
 *     in the signaled state until a thread attempts to wait (at which
 *     time it will unsignal atomicly and return immediately) or
 *     event_unsignal() is called.
*/

void event_init(event_t *, bool initial, uint flags);
void event_destroy(event_t *);

/* Wait until deadline
 * Interruptable arg allows it to return early with ERR_INTERRUPTED if thread
 * is signaled for kill.
 */
status_t event_wait_deadline(event_t *, lk_time_t, bool interruptable);

/* no deadline, non interruptable version of the above. */
static inline status_t event_wait(event_t *e) { return event_wait_deadline(e, INFINITE_TIME, false); }

int event_signal_etc(event_t *, bool reschedule, status_t result);
int event_signal(event_t *, bool reschedule);
int event_signal_thread_locked(event_t *);
status_t event_unsignal(event_t *);

static inline bool event_initialized(const event_t *e) { return e->magic == EVENT_MAGIC; }

static inline bool event_signaled(const event_t *e) { return e->signaled; }

__END_CDECLS;

#endif

