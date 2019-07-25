// Copyright 2016 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include "tests.h"

#include <mxalloc/new.h>
#include <mxtl/unique_ptr.h>
#include <unittest.h>

static bool alloc_checker_ctor(void* context) {
    BEGIN_TEST;

    {
        AllocChecker ac;
    }

    {
        AllocChecker ac;
        ac.check();
    }

    END_TEST;
}

static bool alloc_checker_basic(void* context) {
    BEGIN_TEST;

    AllocChecker ac;
    ac.arm(8u, true);
    EXPECT_TRUE(ac.check(), "");

    ac.arm(16u, false);
    EXPECT_FALSE(ac.check(), "");

    // Allocating zero bytes, allways succeeds.
    ac.arm(0u, false);
    EXPECT_TRUE(ac.check(), "");

    END_TEST;
}

static bool alloc_checker_panic(void* context) {
    BEGIN_TEST;
    // Enable any of the blocks below to test the possible panics.

#if 0
    // Arm but not check should panic (true).
    {
        AllocChecker ac;
        ac.arm(24u, true);
    }
#endif

#if 0
    // Arm but not check should panic (false).
    {
        AllocChecker ac;
        ac.arm(24u, false);
    }
#endif

#if 0
    // Arming twice without a check should panic.
    {
        AllocChecker ac;
        ac.arm(24u, true);
        ac.arm(18u, true);
    }
#endif

    END_TEST;
}

static bool alloc_checker_new(void* context) {
    BEGIN_TEST;

    AllocChecker ac;
    mxtl::unique_ptr<char[]> arr(new (&ac) char[128]);
    EXPECT_EQ(ac.check(), true, "");

    END_TEST;
}

struct BigStruct {
    int x = 5;
    int y[128 * 1024 * 1024];
    int z = 0;
};


UNITTEST_START_TESTCASE(alloc_checker)
UNITTEST("alloc checker ctor & dtor",   alloc_checker_ctor)
UNITTEST("alloc checker basic",         alloc_checker_basic)
UNITTEST("alloc checker panic",         alloc_checker_panic)
UNITTEST("alloc checker new",           alloc_checker_new)
UNITTEST_END_TESTCASE(alloc_checker, "alloc_cpp", "Tests of the C++ AllocChecker", nullptr, nullptr);
