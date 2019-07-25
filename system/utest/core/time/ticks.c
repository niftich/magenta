// Copyright 2016 The Fuchsia Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <magenta/syscalls.h>
#include <unittest/unittest.h>
#include <inttypes.h>

// Calculation of elapsed time using ticks.
static bool elapsed_time_using_ticks(void) {
    BEGIN_TEST;

    uint64_t per_second = mx_ticks_per_second();
    ASSERT_GT(per_second, 0u, "Invalid ticks per second");
    unittest_printf("Ticks per second: %" PRIu64 "\n", per_second);

    uint64_t x = mx_ticks_get();
    uint64_t y = mx_ticks_get();
    ASSERT_GE(y, x, "Ticks went backwards");

    double seconds = (y - x) / (double)per_second;
    ASSERT_GE(seconds, 0u, "Time went backwards");

    END_TEST;
}

BEGIN_TEST_CASE(ticks_tests)
RUN_TEST(elapsed_time_using_ticks)
END_TEST_CASE(ticks_tests)

#ifndef BUILD_COMBINED_TESTS
int main(int argc, char** argv) {
    return unittest_run_all_tests(argc, argv) ? 0 : -1;
}
#endif
