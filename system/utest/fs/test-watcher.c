// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <assert.h>
#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <magenta/device/vfs.h>
#include <magenta/compiler.h>
#include <magenta/syscalls.h>

#include "filesystems.h"
#include "misc.h"

// Try to read from the channel when it should be empty.
bool check_for_empty(mx_handle_t h) {
    char name[NAME_MAX + 1];
    ASSERT_EQ(mx_channel_read(h, 0, &name, NULL, sizeof(name), 0, NULL, NULL),
              ERR_SHOULD_WAIT, "");
    return true;
}

// Try to read the 'expected' name off the channel.
bool check_for_watched(mx_handle_t h, const char* expected, size_t expected_len) {
    mx_signals_t observed;
    ASSERT_EQ(mx_object_wait_one(h, MX_CHANNEL_READABLE,
                                 mx_deadline_after(MX_SEC(5)), &observed),
              NO_ERROR, "");
    ASSERT_EQ(observed & MX_CHANNEL_READABLE, MX_CHANNEL_READABLE, "");
    uint32_t actual;
    char name[NAME_MAX + 1];
    ASSERT_EQ(mx_channel_read(h, 0, &name, NULL, expected_len, 0, &actual, NULL),
              NO_ERROR, "");
    ASSERT_EQ(actual, expected_len, "");
    ASSERT_EQ(strncmp(name, expected, expected_len), 0, "");
    return true;
}

bool test_watcher_basic(void) {
    if (!test_info->supports_watchers) {
        return true;
    }
    BEGIN_TEST;

    ASSERT_EQ(mkdir("::dir", 0666), 0, "");
    DIR* dir = opendir("::dir");
    ASSERT_NONNULL(dir, "");
    mx_handle_t h;
    ASSERT_EQ(ioctl_vfs_watch_dir(dirfd(dir), &h), (ssize_t) sizeof(mx_handle_t), "");

    // The channel should be empty
    ASSERT_TRUE(check_for_empty(h), "");

    // Creating a file in the directory should trigger the watcher
    int fd = open("::dir/foo", O_RDWR | O_CREAT);
    ASSERT_GT(fd, 0, "");
    ASSERT_EQ(close(fd), 0, "");
    ASSERT_TRUE(check_for_watched(h, "foo", sizeof("foo") - 1), "");

    // Renaming into directory should trigger the watcher
    ASSERT_EQ(rename("::dir/foo", "::dir/bar"), 0, "");
    ASSERT_TRUE(check_for_watched(h, "bar", sizeof("bar") - 1), "");

    // Linking into directory should trigger the watcher
    ASSERT_EQ(link("::dir/bar", "::dir/blat"), 0, "");
    ASSERT_TRUE(check_for_watched(h, "blat", sizeof("blat") - 1), "");

    // Clean up
    ASSERT_EQ(unlink("::dir/bar"), 0, "");
    ASSERT_EQ(unlink("::dir/blat"), 0, "");

    // There shouldn't be anything else sitting around on the channel
    ASSERT_TRUE(check_for_empty(h), "");
    ASSERT_EQ(mx_handle_close(h), 0, "");

    ASSERT_EQ(closedir(dir), 0, "");
    ASSERT_EQ(rmdir("::dir"), 0, "");

    END_TEST;
}

RUN_FOR_ALL_FILESYSTEMS(directory_watcher_tests,
    RUN_TEST_MEDIUM(test_watcher_basic)
)
