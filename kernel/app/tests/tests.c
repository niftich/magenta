// Copyright 2016 The Fuchsia Authors
// Copyright (c) 2008 Travis Geiselbrecht
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <app.h>
#include <debug.h>
#include <app/tests.h>
#include <compiler.h>

#if defined(WITH_LIB_CONSOLE)
#include <lib/console.h>

#include <assert.h>
#include <err.h>
#include <platform.h>
#include <unittest.h>

STATIC_COMMAND_START
STATIC_COMMAND("printf_tests", "test printf", (console_cmd)&printf_tests)
STATIC_COMMAND("thread_tests", "test the scheduler", (console_cmd)&thread_tests)
STATIC_COMMAND("port_tests", "test the ports", (console_cmd)&port_tests)
STATIC_COMMAND("clock_tests", "test clocks", (console_cmd)&clock_tests)
STATIC_COMMAND("bench", "miscellaneous benchmarks", (console_cmd)&benchmarks)
STATIC_COMMAND("fibo", "threaded fibonacci", (console_cmd)&fibo)
STATIC_COMMAND("spinner", "create a spinning thread", (console_cmd)&spinner)
STATIC_COMMAND("ref_ptr_tests", "test ref_ptr", (console_cmd)&ref_ptr_tests)
STATIC_COMMAND("ref_counted_tests", "test ref_counted", (console_cmd)&ref_counted_tests)
STATIC_COMMAND("forward_tests", "test forward", (console_cmd)&forward_tests)
STATIC_COMMAND("list_tests", "test lists", (console_cmd)&list_tests)
STATIC_COMMAND("auto_call_tests", "test auto call", (console_cmd)&auto_call_tests)
STATIC_COMMAND("sync_ipi_tests", "test synchronous IPIs", (console_cmd)&sync_ipi_tests)
STATIC_COMMAND_END(tests);

#endif

static void tests_init(const struct app_descriptor *app)
{
}

APP_START(tests)
.init = tests_init,
.flags = 0,
APP_END

