// Copyright 2017 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "tests.h"

#include <stdio.h>
#include <err.h>
#include <inttypes.h>
#include <kernel/timer.h>
#include <kernel/event.h>
#include <kernel/thread.h>
#include <platform.h>

static enum handler_return timer_cb(struct timer* timer, lk_time_t now, void* arg)
{
    event_t* event = (event_t*)arg;
    event_signal(event, false);

    return INT_RESCHEDULE;
}

static int timer_do_one_thread(void* arg)
{
    event_t event;
    timer_t timer;

    event_init(&event, false, 0);
    timer_initialize(&timer);

    timer_set_oneshot(&timer, current_time() + LK_MSEC(10), timer_cb, &event);
    event_wait(&event);

    printf("got timer on cpu %u\n", arch_curr_cpu_num());

    event_destroy(&event);

    return 0;
}

#if WITH_SMP
static thread_t _timer_threads[SMP_MAX_CPUS];
#define timer_threads(cpu) (&_timer_threads[cpu])
#else
static thread_t _timer_thread;
#define timer_threads(cpu) (&_timer_thread)
#endif

static void timer_test_all_cpus(void)
{
    uint max = arch_max_num_cpus();
    uint i;
    for (i = 0; i < max; i++) {
        char name[16];
        snprintf(name, sizeof(name), "timer %u\n", i);

        thread_t* t = thread_create_etc(
                timer_threads(i), name, timer_do_one_thread, NULL,
                DEFAULT_PRIORITY, NULL, NULL, DEFAULT_STACK_SIZE, NULL);
        if (t == NULL) {
            printf("failed to create thread for cpu %d\n", i);
            return;
        }
        thread_set_pinned_cpu(t, i);
        thread_resume(t);
    }
    uint joined = 0;
    for (i = 0; i < max; i++) {
        if (thread_join(timer_threads(i), NULL, LK_SEC(1)) == 0) {
            joined += 1;
        }
    }
    printf("%u threads created, %u threads joined\n", max, joined);
}

void timer_tests(void)
{
    // timer fires on all cpus
    timer_test_all_cpus();
}
