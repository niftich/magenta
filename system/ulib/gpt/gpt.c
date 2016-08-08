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

#include <gpt/gpt.h>
#include <cksum/cksum.h>
#include <ddk/hexdump.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "gpt_defs.h"

// TODO:
// * more error checking

typedef struct gpt_priv {
    int fd;
    size_t blocksize;
    // block size in bytes
    uint64_t blocks;
    // number of blocks
    gpt_header_t header;
    // cached header
    gpt_partition_t ptable[PARTITIONS_COUNT];
    // cached partition table
    int ptable_count;
    // number of actual partitions
    gpt_device_t device;
} gpt_priv_t;

#define get_priv(dev) ((gpt_priv_t*)((uintptr_t)(dev)-offsetof(gpt_priv_t, device)))

static void cstring_to_utf16(uint16_t* dst, const char* src, size_t maxlen) {
    size_t len = strlen(src);
    if (len > maxlen) len = maxlen;
    for (size_t i = 0; i < len; i++) {
        *dst++ = (uint16_t)(*src++ & 0x7f);
    }
}

int gpt_device_init(int fd, size_t blocksize, uint64_t blocks, gpt_device_t** out_dev) {
    gpt_priv_t* priv = calloc(1, sizeof(gpt_priv_t));
    if (!priv) return -1;

    priv->fd = fd;
    priv->blocksize = blocksize;
    priv->blocks = blocks;

    if (priv->blocksize != 512) {
        printf("blocksize != 512 not supported\n");
        return -1;
    }

    // read the gpt header (lba 1)
    int rc = lseek(fd, blocksize, SEEK_SET);
    if (rc < 0) {
        goto fail;
    }

    gpt_header_t* header = &priv->header;
    rc = read(fd, header, blocksize);
    if (rc != blocksize) {
        goto fail;
    }

    printf("header:\n");
    hexdump8(header, sizeof(gpt_header_t));

    // is this a valid gpt header?
    if (header->magic != GPT_MAGIC) {
        printf("invalid header magic!\n");
        goto out; // ok to have an invalid header
    }

    // header checksum
    uint32_t saved_crc = header->crc32;
    header->crc32 = 0;
    uint32_t crc = crc32(0, (const unsigned char*)header, header->size);
    if (crc != saved_crc) {
        printf("header crc check failed\n");
        goto out;
    }

    if (header->entries_count > PARTITIONS_COUNT) {
        printf("too many partitions!\n");
        goto out;
    }

    priv->device.valid = true;

    if (header->entries_count == 0) {
        goto out;
    }
    if (header->entries_count > PARTITIONS_COUNT) {
        printf("too many partitions\n");
        goto out;
    }

    gpt_partition_t* ptable = priv->ptable;

    // read the partition table
    rc = lseek(fd, header->entries * blocksize, SEEK_SET);
    if (rc < 0) {
        goto fail;
    }
    ssize_t ptable_size = header->entries_size * header->entries_count;
    if ((size_t)ptable_size > SIZE_MAX) {
        printf("partition table too big\n");
        goto out;
    }
    rc = read(fd, ptable, ptable_size);
    if (rc != ptable_size) {
        goto fail;
    }

    // partition table checksum
    crc = crc32(0, (const unsigned char*)ptable, ptable_size);
    if (crc != header->entries_crc) {
        printf("table crc check failed\n");
        goto out;
    }

    // keep the partition table sorted
    gpt_partition_t* tmp;
    gpt_partition_t** sorted = priv->device.partitions;
    for (unsigned i = 0; i < header->entries_count; i++) {
        if (ptable[i].first == 0 && ptable[i].last == 0) continue;
        sorted[i] = &ptable[i];
        priv->ptable_count += 1;
        for (int j = i; j > 0; j--) {
            if (sorted[j - 1]->last < sorted[j]->first) {
                break;
            } else {
                tmp = sorted[j - 1];
                sorted[j - 1] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }
out:
    *out_dev = &priv->device;
    return 0;
fail:
    free(priv);
    return -1;
}

void gpt_device_release(gpt_device_t* dev) {
    gpt_priv_t* priv = get_priv(dev);
    free(priv);
}

static int gpt_sync_current(int fd, int blocksize, gpt_header_t* header, gpt_partition_t* ptable) {
    // write partition table first
    ssize_t rc = lseek(fd, header->entries * blocksize, SEEK_SET);
    if (rc < 0) {
        return -1;
    }
    size_t ptable_size = header->entries_count * header->entries_size;
    printf("table: offset=%llu size=%zu\n", header->entries, ptable_size);
    hexdump8(ptable, ptable_size);
    rc = write(fd, ptable, ptable_size);
    if (rc < 0 || (size_t)rc != ptable_size) {
        return -1;
    }
    // then write the header
    rc = lseek(fd, header->current * blocksize, SEEK_SET);
    if (rc < 0) {
        return -1;
    }
    printf("header:\n");
    hexdump8(header, sizeof(gpt_header_t));
    rc = write(fd, header, sizeof(gpt_header_t));
    if (rc != sizeof(gpt_header_t)) {
        return -1;
    }
    return 0;
}

int gpt_device_sync(gpt_device_t* dev) {
    gpt_priv_t* priv = get_priv(dev);

    // fill in the new header fields
    gpt_header_t header;
    memset(&header, 0, sizeof(gpt_header_t));
    header.magic = GPT_MAGIC;
    header.revision = 0x00010000; // gpt version 1.0
    header.size = GPT_HEADER_SIZE;
    if (dev->valid) {
        header.current = priv->header.current;
        header.backup = priv->header.backup;
        memcpy(header.guid, priv->header.guid, 16);
    } else {
        header.current = 1;
        // backup gpt is in the last block
        header.backup = priv->blocks - 1;
        // generate a guid
    }

    // always write 128 entries in partition table
    size_t ptable_size = PARTITIONS_COUNT * sizeof(gpt_partition_t);
    void* buf = malloc(ptable_size);
    if (!buf) {
        return -1;
    }
    memset(buf, 0, ptable_size);

    // generate partition table
    void* ptr = buf;
    for (int i = 0, gpt_partition_t* p = dev->partitions; i < PARTITIONS_COUNT && p; i++, p++) {
        memcpy(ptr, p, GPT_ENTRY_SIZE);
        ptr += GPT_ENTRY_SIZE;
    }

    // fill in partition table fields in header
    header.entries = dev->valid ? priv->header.entries : 2;
    header.entries_count = PARTITIONS_COUNT;
    header.entries_size = GPT_ENTRY_SIZE;
    header.entries_crc = crc32(0, buf, ptable_size);

    size_t ptable_blocks = ptable_size / priv->blocksize;
    header.first = header.entries + ptable_blocks;
    header.last = header.backup - ptable_blocks - 1;

    // calculate header checksum
    header.crc32 = crc32(0, (const unsigned char*)&header, GPT_HEADER_SIZE);

    // the copy cached in priv is the primary copy
    memcpy(&priv->header, &header, GPT_HEADER_SIZE);
    dev->valid = true;

    // the header copy on stack is now the backup copy...
    header.current = priv->header.backup;
    header.backup = priv->header.current;
    header.entries = priv->header.last + 1;
    header.crc32 = 0;
    header.crc32 = crc32(0, (const unsigned char*)&header, GPT_HEADER_SIZE);

    // write backup to disk
    int rc = gpt_sync_current(priv->fd, priv->blocksize, &header, buf);
    if (rc < 0) {
        goto fail;
    }

    // write primary copy to disk
    rc = gpt_sync_current(priv->fd, priv->blocksize, &priv->header, buf);
    if (rc < 0) {
        goto fail;
    }

    free(buf);
    return 0;
fail:
    free(buf);
    return -1;
}

int gpt_partition_add(gpt_device_t* dev, const char* name, uint8_t* type, uint8_t* guid, uint64_t offset, uint64_t blocks, uint64_t flags) {
    gpt_priv_t* priv = get_priv(dev);
    if (priv->partitions_count == 128) {
        printf("too many partitions\n");
        return -1;
    }

    if (offset > priv->blocks || offset + blocks > priv->blocks) {
        printf("offset is too big!\n");
        return -1;
    }

    // find a free slot
    gpt_partition_t* part = NULL;
    for (int i = 0; i < 128; i++) {
        if (priv->entries[i].first == 0 && priv->entries[i].last == 0) {
            part = &priv->entries[i];
            break;
        }
    }

    // insert the new element into the list
    gpt_partition_t** table = dev->partitions;
    for (int i = priv->partitions_count - 1; i >= 0; i--) {
        if (table[i]->last < offset) {
            memcpy(part->type, type, sizeof(part->type));
            memcpy(part->guid, guid, sizeof(part->guid));
            part->first = offset;
            part->last = offset + blocks;
            part->flags = flags;
            cstring_to_utf16((uint16_t*)part->name, name, sizeof(part->name) / sizeof(uint16_t));
            table[i+1] = part;
        }
    }

    priv->partitions_count += 1;
    return 0;
}

int gpt_partition_remove(gpt_device_t* dev, const uint8_t* guid) {
    gpt_priv_t* priv = get_priv(dev);
    // look for the entry in the partition list
    int i;
    for (i = 0; i < priv->partitions_count; i++) {
        if (!memcmp(dev->partitions[i]->guid, guid, sizeof(dev->partitions[i]->guid))) {
            break;
        }
    }
    if (i == priv->partitions_count) {
        printf("partition not found\n");
        return -1;
    }
    // clear the entry
    memset(dev->partitions[i], 0, sizeof(gpt_partition_t));
    // pack the partition list
    for (i = i + 1; i < priv->partitions_count; i++) {
        dev->partitions[i-1] = dev->partitions[i];
    }
    dev->partitions[priv->partitions_count-1] = NULL;
    priv->partitions_count -= 1;
    return 0;
}
