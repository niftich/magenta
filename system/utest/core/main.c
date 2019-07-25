// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/uio.h>

#include <magenta/processargs.h>
#include <magenta/syscalls.h>
#include <magenta/syscalls/log.h>

#include <unittest/unittest.h>

// output via debuglog syscalls

static mx_handle_t log_handle;

#define LOGBUF_MAX (MX_LOG_RECORD_MAX - sizeof(mx_log_record_t))

static void log_write(const void* data, size_t len) {
    while (len > 0) {
        size_t xfer = (len > LOGBUF_MAX) ? LOGBUF_MAX : len;
        mx_log_write(log_handle, xfer, data, 0);
        data += xfer;
        len -= xfer;
    }
}


// libc init and io stubs
// The reason these are here is that the "core" tests intentionally do not
// use mxio. See ./README.md.

mx_handle_t root_resource;

void __libc_extensions_init(uint32_t count, mx_handle_t handle[], uint32_t info[]) {
    for (unsigned n = 0; n < count; n++) {
        if (info[n] == PA_HND(PA_RESOURCE, 0)) {
            root_resource = handle[n];
            handle[n] = 0;
            info[n] = 0;
            break;
        }
    }
}

mx_handle_t get_root_resource(void) {
    return root_resource;
}

ssize_t write(int fd, const void* data, size_t count) {
    if ((fd == 1) || (fd == 2)) {
        log_write(data, count);
    }
    return count;
}

ssize_t readv(int fd, const struct iovec* iov, int num) {
    return 0;
}

ssize_t writev(int fd, const struct iovec* iov, int num) {
    ssize_t count = 0;
    ssize_t r;
    while (num > 0) {
        if (iov->iov_len != 0) {
            r = write(fd, iov->iov_base, iov->iov_len);
            if (r < 0) {
                return count ? count : r;
            }
            if ((size_t)r < iov->iov_len) {
                return count + r;
            }
            count += r;
        }
        iov++;
        num--;
    }
    return count;
}

#define ERROR() do { errno = ENOSYS; return -1; } while (0)

off_t lseek(int fd, off_t offset, int whence) {
    ERROR();
}

int isatty(int fd) {
    return 1;
}

int main(int argc, char** argv) {
    if (mx_log_create(0, &log_handle) < 0) {
        return -2;
    }
    mx_log_write(log_handle, 4, "TEST", 0);
    return unittest_run_all_tests(argc, argv) ? 0 : -1;
}
