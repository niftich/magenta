// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <unittest/unittest.h>
#include <sched.h>
#include <stdbool.h>

static bool dso_ctor_ran;

static struct Global {
    Global() { dso_ctor_ran = true; }
    ~Global() {
        // This is just some random nonempty thing that the compiler
        // can definitely never decide to optimize away.  We can't
        // easily test that the destructor got run, but we can ensure
        // that using a static destructor compiles and links correctly.
        sched_yield();
    }
} global;

bool check_dso_ctor() {
    BEGIN_HELPER;
    EXPECT_TRUE(dso_ctor_ran, "DSO global constuctor didn't run!");
    END_HELPER;
}
