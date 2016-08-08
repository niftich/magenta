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

#include <ddk/hexdump.h>
#include <ddk/protocol/block.h>
#include <gpt/gpt.h>
#include <magenta/syscalls.h> // for mx_cprng_draw
#include <mxio/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#define DEFAULT_BLOCKDEV "/dev/class/block/000"

static int cgetc(void) {
    uint8_t ch;
    for (;;) {
        mxio_wait_fd(0, MXIO_EVT_READABLE, NULL, MX_TIME_INFINITE);
        int r = read(0, &ch, 1);
        if (r < 0) return r;
        if (r == 1) return ch;
    }
}

static char* utf16_to_cstring(char* dst, const uint16_t* src, size_t len) {
    size_t i = 0;
    char* ptr = dst;
    while (i < len) {
        char c = src[i++] & 0x7f;
        if (!c) continue;
        *ptr++ = c;
    }
    return dst;
}

static gpt_device_t* init(const char* dev, int* out_fd) {
    printf("Using %s... <enter> to continue, any other key to cancel\n", dev);

    int c = cgetc();
    if (c != 10) return NULL;

    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        printf("error opening %s\n", dev);
        return NULL;
    }

    uint64_t blocksize;
    int rc = mxio_ioctl(fd, BLOCK_OP_GET_BLOCKSIZE, NULL, 0, &blocksize, sizeof(blocksize));
    if (rc < 0) {
        printf("error getting block size\n");
        close(fd);
        return NULL;
    }

    uint64_t blocks;
    rc = mxio_ioctl(fd, BLOCK_OP_GET_SIZE, NULL, 0, &blocks, sizeof(blocks));
    if (rc < 0) {
        printf("error getting device size\n");
        close(fd);
        return NULL;
    }
    blocks /= blocksize;

    printf("blocksize=%llu blocks=%llu\n", blocksize, blocks);

    gpt_device_t* gpt;
    rc = gpt_device_init(fd, blocksize, blocks, &gpt);
    if (rc < 0) {
        printf("error initializing test\n");
        close(fd);
        return NULL;
    }

    *out_fd = fd;
    return gpt;
}

static void dump_partitions(const char* dev) {
    int fd;
    gpt_device_t* gpt = init(dev, &fd);
    if (!gpt) return;

    if (!gpt->valid) {
        printf("No valid GPT found\n");
        return;
    }

    printf("Partition table is valid\n");
    printf("Partitions:\n");
    gpt_partition_t* p;
    char name[37];
    int i;
    for (i = 0; i < MAX_PARTITIONS_COUNT; i++) {
        p = gpt->partitions[i];
        if (!p) break;
        memset(name, 0, 37);
        printf("%d: %s %llu %llu (%llu blocks)\n", i, utf16_to_cstring(name, (const uint16_t*)p->name, 36), p->first, p->last, p->last - p->first + 1);
    }
    printf("Total: %d partitions\n", i);

    gpt_device_release(gpt);
    close(fd);
}

static void add_partition(const char* dev, uint64_t offset, uint64_t blocks, const char* name) {
    int fd;
    gpt_device_t* gpt = init(dev, &fd);
    if (!gpt) return;

    uint8_t type[16];
    uint8_t guid[16];
    memset(type, 0xf, 16);
    mx_cprng_draw(guid, 16);
    gpt_partition_add(gpt, name, type, guid, offset, blocks, 0);
    gpt_device_sync(gpt);

    gpt_device_release(gpt);
    close(fd);
}

static void remove_partition(const char* dev, const uint8_t* guid) {
}

static uint64_t argtoull(const char* arg) {
    int base = 10;
    if (arg[0] == '0' && (arg[1] == 'x' || arg[1] == 'X')) {
        base = 16;
    }
    return strtoull(arg, NULL, base);
}

int main(int argc, char** argv) {
    const char* dev = DEFAULT_BLOCKDEV;
    if (argc == 1) goto usage;

    const char* cmd = argv[1];
    if (!strcmp(cmd, "dump")) {
        dump_partitions(argc > 2 ? argv[2] : dev);
    } else if (!strcmp(cmd, "add")) {
        if (argc < 5) goto usage;
        add_partition(argc > 5 ? argv[5] : dev, argtoull(argv[2]), argtoull(argv[3]), argv[4]);
    } else if (!strcmp(cmd, "remove")) {
        if (argc < 3) goto usage;
        remove_partition(argc > 3 ? argv[3] : dev, (const uint8_t*)argv[2]);
    } else {
        goto usage;
    }

    return 0;
usage:
    printf("usage:\n");
    printf("dump [<dev>]\n");
    printf("add <offset> <blocks> <name> [<dev>]\n");
    printf("remove <guid> [<dev>]\n");
    return 0;
}
