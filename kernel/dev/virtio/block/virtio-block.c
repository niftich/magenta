// Copyright 2016 The Fuchsia Authors
// Copyright (c) 2014-2015 Travis Geiselbrecht
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <debug.h>
#include <assert.h>
#include <trace.h>
#include <compiler.h>
#include <string.h>
#include <list.h>
#include <err.h>
#include <kernel/thread.h>
#include <kernel/event.h>
#include <kernel/mutex.h>
#include <kernel/vm.h>
#include <lib/bio.h>
#include <dev/virtio.h>

#if WITH_DEV_PCIE
#include <dev/pcie.h>
#endif

#define LOCAL_TRACE 1

struct virtio_blk_config {
    uint64_t capacity;
    uint32_t size_max;
    uint32_t seg_max;
    struct virtio_blk_geometry {
        uint16_t cylinders;
        uint8_t heads;
        uint8_t sectors;
    } geometry;
    uint32_t blk_size;
} __PACKED;

struct virtio_blk_req {
    uint32_t type;
    uint32_t ioprio;
    uint64_t sector;
} __PACKED;

#define VIRTIO_BLK_F_BARRIER  (1<<0)
#define VIRTIO_BLK_F_SIZE_MAX (1<<1)
#define VIRTIO_BLK_F_SEG_MAX  (1<<2)
#define VIRTIO_BLK_F_GEOMETRY (1<<4)
#define VIRTIO_BLK_F_RO       (1<<5)
#define VIRTIO_BLK_F_BLK_SIZE (1<<6)
#define VIRTIO_BLK_F_SCSI     (1<<7)
#define VIRTIO_BLK_F_FLUSH    (1<<9)
#define VIRTIO_BLK_F_TOPOLOGY (1<<10)
#define VIRTIO_BLK_F_CONFIG_WCE (1<<11)

#define VIRTIO_BLK_T_IN         0
#define VIRTIO_BLK_T_OUT        1
#define VIRTIO_BLK_T_FLUSH      4

#define VIRTIO_BLK_S_OK         0
#define VIRTIO_BLK_S_IOERR      1
#define VIRTIO_BLK_S_UNSUPP     2

static enum handler_return virtio_block_irq_driver_callback(struct virtio_device *dev, uint ring, const struct vring_used_elem *e);
static ssize_t virtio_bdev_read_block(struct bdev *bdev, void *buf, bnum_t block, uint count);
static ssize_t virtio_bdev_write_block(struct bdev *bdev, const void *buf, bnum_t block, uint count);
static status_t virtio_block_init(struct virtio_device *dev);

struct virtio_block_dev {
    struct virtio_device *dev;

    mutex_t lock;
    event_t io_event;

    /* bio block device */
    bdev_t bdev;

    /* one blk_req structure for io, not crossing a page boundary */
    struct virtio_blk_req *blk_req;
    paddr_t blk_req_phys;

    /* one uint8_t response word */
    uint8_t blk_response;
    paddr_t blk_response_phys;
};

VIRTIO_DEV_CLASS(block, VIRTIO_DEV_ID_BLOCK, NULL, virtio_block_init, NULL);

static status_t virtio_block_init(struct virtio_device *dev)
{
    LTRACEF("dev %p\n", dev);

    /* allocate a new block device */
    struct virtio_block_dev *bdev = malloc(sizeof(struct virtio_block_dev));
    if (!bdev)
        return ERR_NO_MEMORY;

    mutex_init(&bdev->lock);
    event_init(&bdev->io_event, false, EVENT_FLAG_AUTOUNSIGNAL);

    bdev->dev = dev;
    dev->priv = bdev;

    bdev->blk_req = memalign(sizeof(struct virtio_blk_req), sizeof(struct virtio_blk_req));
#if WITH_KERNEL_VM
    bdev->blk_req_phys = vaddr_to_paddr(bdev->blk_req);
#else
    bdev->blk_req_phys = (paddr_t)bdev->blk_req;
#endif
    LTRACEF("blk_req structure at %p (0x%lx phys)\n", bdev->blk_req, bdev->blk_req_phys);

#if WITH_KERNEL_VM
    bdev->blk_response_phys = vaddr_to_paddr(&bdev->blk_response);
#else
    bdev->blk_response_phys = (paddr_t)&bdev->blk_response;
#endif

    /* make sure the device is reset */
    virtio_reset_device(dev);

    struct virtio_blk_config config;

    if (dev->type == VIO_MMIO) {
        memcpy(&config, dev->mmio.config_ptr, sizeof(config));
    } else if (dev->type == VIO_PCI) {
        virtio_pci_copy_device_config(dev, &config, sizeof(config));
    }

    LTRACEF("capacity 0x%llx\n", config.capacity);
    LTRACEF("size_max 0x%x\n", config.size_max);
    LTRACEF("seg_max  0x%x\n", config.seg_max);
    LTRACEF("blk_size 0x%x\n", config.blk_size);

    /* ack and set the driver status bit */
    virtio_status_acknowledge_driver(dev);

    // XXX check features bits and ack/nak them

    /* allocate a virtio ring */
    status_t err = virtio_alloc_ring(dev, 0, 128); // XXX set to 128 to match legacy pci
    if (err < 0)
        panic("failed to allocate virtio ring\n");

    /* set our irq handler */
    dev->irq_driver_callback = &virtio_block_irq_driver_callback;

    /* set DRIVER_OK */
    virtio_status_driver_ok(dev);

    /* construct the block device */
    static uint8_t found_index = 0;
    char buf[16];
    snprintf(buf, sizeof(buf), "virtio%u", found_index++);
    bio_initialize_bdev(&bdev->bdev, buf,
                        config.blk_size, config.capacity,
                        0, NULL, BIO_FLAGS_NONE);

    /* override our block device hooks */
    bdev->bdev.read_block = &virtio_bdev_read_block;
    bdev->bdev.write_block = &virtio_bdev_write_block;

    bio_register_device(&bdev->bdev);

    printf("found virtio block device of size %lld\n", config.capacity * config.blk_size);

    return NO_ERROR;
}

static enum handler_return virtio_block_irq_driver_callback(struct virtio_device *dev, uint ring, const struct vring_used_elem *e)
{
    struct virtio_block_dev *bdev = (struct virtio_block_dev *)dev->priv;

    LTRACEF("dev %p, ring %u, e %p, id %u, len %u\n", dev, ring, e, e->id, e->len);

    /* parse our descriptor chain, add back to the free queue */
    uint16_t i = e->id;
    for (;;) {
        int next;
        struct vring_desc *desc = virtio_desc_index_to_desc(dev, ring, i);

        //virtio_dump_desc(desc);

        if (desc->flags & VRING_DESC_F_NEXT) {
            next = desc->next;
        } else {
            /* end of chain */
            next = -1;
        }

        virtio_free_desc(dev, ring, i);

        if (next < 0)
            break;
        i = next;
    }

    /* signal our event */
    event_signal(&bdev->io_event, false);

    return INT_RESCHEDULE;
}

ssize_t virtio_block_read_write(struct virtio_device *dev, void *buf, off_t offset, size_t len, bool write)
{
    struct virtio_block_dev *bdev = (struct virtio_block_dev *)dev->priv;

    uint16_t i;
    struct vring_desc *desc;
    paddr_t pa;
    vaddr_t va = (vaddr_t)buf;

    LTRACEF("dev %p, buf %p, offset 0x%llx, len %zu\n", dev, buf, offset, len);

    mutex_acquire(&bdev->lock);

    /* set up the request */
    bdev->blk_req->type = write ? VIRTIO_BLK_T_OUT : VIRTIO_BLK_T_IN;
    bdev->blk_req->ioprio = 0;
    bdev->blk_req->sector = offset / 512;
    LTRACEF("blk_req type %u ioprio %u sector %llu\n",
            bdev->blk_req->type, bdev->blk_req->ioprio, bdev->blk_req->sector);

    /* put together a transfer */
    desc = virtio_alloc_desc_chain(dev, 0, 3, &i);
    LTRACEF("after alloc chain desc %p, i %u\n", desc, i);

    // XXX not cache safe.
    // At the moment only tested on arm qemu, which doesn't emulate cache.

    /* set up the descriptor pointing to the head */
    desc->addr = bdev->blk_req_phys;
    desc->len = sizeof(struct virtio_blk_req);
    desc->flags |= VRING_DESC_F_NEXT;

    /* set up the descriptor pointing to the buffer */
    desc = virtio_desc_index_to_desc(dev, 0, desc->next);
#if WITH_KERNEL_VM
    /* translate the first buffer */
    pa = vaddr_to_paddr((void *)va);
    // XXX check for error
    desc->addr = (uint64_t)pa;
    /* desc->len is filled in below */
#else
    desc->addr = (uint64_t)(uintptr_t)buf;
    desc->len = len;
#endif
    desc->flags |= write ? 0 : VRING_DESC_F_WRITE; /* mark buffer as write-only if its a block read */
    desc->flags |= VRING_DESC_F_NEXT;

#if WITH_KERNEL_VM
    /* see if we need to add more descriptors due to scatter gather */
    paddr_t next_pa = PAGE_ALIGN(pa + 1);
    desc->len = MIN(next_pa - pa, len);
    LTRACEF("first descriptor va 0x%lx desc->addr 0x%llx desc->len %u\n", va, desc->addr, desc->len);
    len -= desc->len;
    while (len > 0) {
        /* amount of source buffer handled by this iteration of the loop */
        size_t len_tohandle = MIN(len, PAGE_SIZE);

        /* translate the next page in the buffer */
        va = PAGE_ALIGN(va + 1);
        pa = vaddr_to_paddr((void *)va);
        // XXX check for error
        LTRACEF("va now 0x%lx, pa 0x%lx, next_pa 0x%lx, remaining len %zu\n", va, pa, next_pa, len);

        /* is the new translated physical address contiguous to the last one? */
        if (next_pa == pa) {
            LTRACEF("extending last one by %zu bytes\n", len_tohandle);
            desc->len += len_tohandle;
        } else {
            uint16_t next_i = virtio_alloc_desc(dev, 0);
            struct vring_desc *next_desc = virtio_desc_index_to_desc(dev, 0, next_i);
            DEBUG_ASSERT(next_desc);

            LTRACEF("doesn't extend, need new desc, allocated desc %i (%p)\n", next_i, next_desc);

            /* fill this descriptor in and put it after the last one but before the response descriptor */
            next_desc->addr = (uint64_t)pa;
            next_desc->len = len_tohandle;
            next_desc->flags = write ? 0 : VRING_DESC_F_WRITE; /* mark buffer as write-only if its a block read */
            next_desc->flags |= VRING_DESC_F_NEXT;
            next_desc->next = desc->next;
            desc->next = next_i;

            desc = next_desc;
        }
        len -= len_tohandle;
        next_pa += PAGE_SIZE;
    }
#endif

    /* set up the descriptor pointing to the response */
    desc = virtio_desc_index_to_desc(dev, 0, desc->next);
    desc->addr = bdev->blk_response_phys;
    desc->len = 1;
    desc->flags = VRING_DESC_F_WRITE;

    /* submit the transfer */
    virtio_submit_chain(dev, 0, i);

    /* kick it off */
    virtio_kick(dev, 0);

    /* wait for the transfer to complete */
    event_wait(&bdev->io_event);

    LTRACEF("status 0x%hhx\n", bdev->blk_response);

    mutex_release(&bdev->lock);

    return len;
}

static ssize_t virtio_bdev_read_block(struct bdev *bdev, void *buf, bnum_t block, uint count)
{
    struct virtio_block_dev *dev = containerof(bdev, struct virtio_block_dev, bdev);

    LTRACEF("dev %p, buf %p, block 0x%x, count %u\n", bdev, buf, block, count);

    if (virtio_block_read_write(dev->dev, buf, (off_t)block * dev->bdev.block_size,
                                count * dev->bdev.block_size, false) == 0) {
        return count * dev->bdev.block_size;
    } else {
        return ERR_IO;
    }
}

static ssize_t virtio_bdev_write_block(struct bdev *bdev, const void *buf, bnum_t block, uint count)
{
    struct virtio_block_dev *dev = containerof(bdev, struct virtio_block_dev, bdev);

    LTRACEF("dev %p, buf %p, block 0x%x, count %u\n", bdev, buf, block, count);

    if (virtio_block_read_write(dev->dev, (void *)buf, (off_t)block * dev->bdev.block_size,
                                count * dev->bdev.block_size, true) == 0) {
        return count * dev->bdev.block_size;
    } else {
        return ERR_IO;
    }
}

#if WITH_DEV_PCIE
// PCI
#include <dev/pcie.h>

static void* virtio_block_pci_probe(pcie_device_state_t* pci_device)
{
    DEBUG_ASSERT(pci_device);

    /* Is this the droid we are looking for? */
    if ((pci_device->common.vendor_id != 0x1af4) ||
        (pci_device->common.device_id != 0x1001)) {
        return NULL;
    }

    LTRACEF("found our pci device!\n");

    struct virtio_device *dev;
    status_t err = virtio_add_pci_device(&dev, pci_device);
    if (err < 0)
        return NULL;

    return dev;
}

// XXX hack
extern pcie_irq_handler_retval_t virtio_pci_irq_handler(struct pcie_common_state* pci_device, uint  irq_id, void* ctx);

static status_t virtio_block_pci_startup(struct pcie_device_state* pci_device)
{
    struct virtio_device *dev = (struct virtio_device *)pci_device->driver_ctx;

    LTRACE_ENTRY;

    status_t ret;
    pcie_irq_mode_caps_t caps;
    ret = pcie_query_irq_mode_capabilities(&pci_device->common,
                                          PCIE_IRQ_MODE_LEGACY,
                                          &caps);
    LTRACEF("Legacy IRQ: status %d, max irqs %u\n", ret, caps.max_irqs);

    ret = pcie_query_irq_mode_capabilities(&pci_device->common,
                                          PCIE_IRQ_MODE_MSI,
                                          &caps);
    LTRACEF("MSI IRQ: status %d, max irqs %u\n", ret, caps.max_irqs);

    ret = pcie_query_irq_mode_capabilities(&pci_device->common,
                                          PCIE_IRQ_MODE_MSI_X,
                                          &caps);
    LTRACEF("MSI-X IRQ: status %d, max irqs %u\n", ret, caps.max_irqs);

    ret = pcie_set_irq_mode(&dev->pci.pci_state->common,
                            PCIE_IRQ_MODE_LEGACY,
                            1,
                            PCIE_IRQ_SHARE_MODE_SYSTEM_SHARED);
    LTRACEF("pcie_set_irq_mode legacy allocate returns %d\n", ret);

    // call our init routine
    ret = virtio_block_init(dev);
    if (ret != NO_ERROR)
        return ret;

    /* Register our handler; if the mode we are operating in does not support
     * masking, we might start to receive interrupts as soon as we register. */
    ret = pcie_register_irq_handler(&pci_device->common, 0, virtio_pci_irq_handler, dev);
    if (ret != NO_ERROR) {
        TRACEF("Failed to register IRQ handler (err = %d)\n", ret);
        return ret;
    }

    ret = pcie_unmask_irq(&pci_device->common, 0);
    if (ret != NO_ERROR) {
        TRACEF("Failed to unmask IRQ (err = %d)\n", ret);
        return ret;
    }

    return NO_ERROR;
}

static const pcie_driver_fn_table_t virtio_block_pci_fn = {
    .pcie_probe_fn    = virtio_block_pci_probe,
    .pcie_startup_fn  = virtio_block_pci_startup,
    .pcie_shutdown_fn = NULL, //intel_i915_pci_shutdown,
    .pcie_release_fn  = NULL, //intel_i915_pci_release,
};

STATIC_PCIE_DRIVER(virtio_block, "Virtio Block Device", virtio_block_pci_fn);

#endif

