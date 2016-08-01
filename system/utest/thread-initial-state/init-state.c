// Copyright 2016 The Fuchsia Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <assert.h>
#include <magenta/syscalls.h>
#include <unittest/unittest.h>
#include <unittest/test-utils.h>
#include <stdio.h>
#include <system/compiler.h>

extern void thread_entry(uintptr_t arg);

int print_fail(void) {
    EXPECT_TRUE(false, "Failed");
    mx_thread_exit();
    return 1; // Not reached
}

// create a thread using the raw magenta api.
// cannot use a higher level api because they'll use trampoline functions that'll trash
// registers on entry.
mx_handle_t raw_thread_create(void (*thread_entry)(uintptr_t arg), uintptr_t arg)
{
    // preallocated stack to satisfy the thread we create
    static uint8_t stack[1024] __ALIGNED(16);

    // TODO: get current process handle
    mx_handle_t process_self_handle = 0;

    mx_handle_t handle = mx_thread_create(process_self_handle, "", 0, 0);
    if (handle < 0)
        return handle;

    mx_status_t status = mx_thread_start(handle, (uintptr_t)thread_entry, (uintptr_t)stack + sizeof(stack), arg);
    if (status < 0)
        return status;

    return handle;
}

bool tis_test(void) {
    BEGIN_TEST;
#if _LP64
    uintptr_t arg = 0x1234567890abcdef;
#else
    uintptr_t arg = 0x90abcdef;
#endif
    mx_handle_t handle = raw_thread_create(thread_entry, arg);
    ASSERT_GE(handle, 0, "Error while thread creation");

    mx_status_t status = mx_handle_wait_one(handle, MX_SIGNAL_SIGNALED,
                                                  MX_TIME_INFINITE, NULL);
    ASSERT_GE(status, 0, "Error while thread wait");
    END_TEST;
}

BEGIN_TEST_CASE(tis_tests)
RUN_TEST(tis_test)
END_TEST_CASE(tis_tests)

int main(int argc, char** argv) {
    return unittest_run_all_tests(argc, argv) ? 0 : -1;
}
