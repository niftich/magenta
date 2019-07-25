// Copyright 2016 The Fuchsia Authors
// Copyright (c) 2008-2015 Travis Geiselbrecht
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT


#include <ctype.h>
#include <debug.h>
#include <stdlib.h>
#include <printf.h>
#include <stdio.h>
#include <list.h>
#include <arch/ops.h>
#include <dev/hw_rng.h>
#include <platform.h>
#include <platform/debug.h>
#include <kernel/spinlock.h>

void spin(uint32_t usecs)
{
    lk_time_t start = current_time();

    lk_time_t nsecs = (lk_time_t)usecs * 1000;
    while ((current_time() - start) < nsecs)
        ;
}

void _panic(void *caller, void *frame, const char *fmt, ...)
{
    platform_panic_start();

    printf("panic (caller %p frame %p): ", caller, frame);

    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);

    platform_halt(HALT_ACTION_HALT, HALT_REASON_SW_PANIC);
}

void __stack_chk_fail(void) {
    panic("stack canary corrupted!\n");
}

uintptr_t choose_stack_guard(void) {
    uintptr_t guard;
    if (hw_rng_get_entropy(&guard, sizeof(guard), true) != sizeof(guard)) {
        // We can't get a random value, so use a randomish value.
        guard = 0xdeadbeef00ff00ffUL ^ (uintptr_t)&guard;
    }
    return guard;
}

#if !DISABLE_DEBUG_OUTPUT

void hexdump_very_ex(const void *ptr, size_t len, uint64_t disp_addr, hexdump_print_fn_t* pfn)
{
    addr_t address = (addr_t)ptr;
    size_t count;

    for (count = 0 ; count < len; count += 16) {
        union {
            uint32_t buf[4];
            uint8_t  cbuf[16];
        } u;
        size_t s = ROUNDUP(MIN(len - count, 16), 4);
        size_t i;

        pfn(((disp_addr + len) > 0xFFFFFFFF)
               ? "0x%016llx: "
               : "0x%08llx: ", disp_addr + count);

        for (i = 0; i < s / 4; i++) {
            u.buf[i] = ((const uint32_t *)address)[i];
            pfn("%08x ", u.buf[i]);
        }
        for (; i < 4; i++) {
            pfn("         ");
        }
        pfn("|");

        for (i=0; i < 16; i++) {
            char c = u.cbuf[i];
            if (i < s && isprint(c)) {
                pfn("%c", c);
            } else {
                pfn(".");
            }
        }
        pfn("|\n");
        address += 16;
    }
}

void hexdump8_very_ex(const void *ptr, size_t len, uint64_t disp_addr, hexdump_print_fn_t* pfn)
{
    addr_t address = (addr_t)ptr;
    size_t count;
    size_t i;

    for (count = 0 ; count < len; count += 16) {
        pfn(((disp_addr + len) > 0xFFFFFFFF)
               ? "0x%016llx: "
               : "0x%08llx: ", disp_addr + count);

        for (i=0; i < MIN(len - count, 16); i++) {
            pfn("%02hhx ", *(const uint8_t *)(address + i));
        }

        for (; i < 16; i++) {
            pfn("   ");
        }

        pfn("|");

        for (i=0; i < MIN(len - count, 16); i++) {
            char c = ((const char *)address)[i];
            pfn("%c", isprint(c) ? c : '.');
        }

        pfn("\n");
        address += 16;
    }
}

#endif // !DISABLE_DEBUG_OUTPUT
