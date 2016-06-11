/*
 * Copyright (c) 2014-2015 Travis Geiselbrecht
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <dev/virtio.h>
#include <dev/virtio/virtio_ring.h>

#include <debug.h>
#include <assert.h>
#include <trace.h>
#include <compiler.h>
#include <list.h>
#include <err.h>
#include <stdlib.h>
#include <string.h>
#include <pow2.h>
#include <lk/init.h>
#include <dev/interrupt.h>

#include "virtio_priv.h"

#define LOCAL_TRACE 0

/* list of class drivers, declared with VIRTIO_DEV_CLASS() macro */
extern const virtio_dev_class_t __start_virtio_classes[] __WEAK;
extern const virtio_dev_class_t __stop_virtio_classes[] __WEAK;

static struct virtio_device *virtio_mmio_devices;

static void dump_mmio_config(const volatile struct virtio_mmio_config *mmio)
{
    printf("mmio at %p\n", mmio);
    printf("\tmagic 0x%x\n", mmio->magic);
    printf("\tversion 0x%x\n", mmio->version);
    printf("\tdevice_id 0x%x\n", mmio->device_id);
    printf("\tvendor_id 0x%x\n", mmio->vendor_id);
    printf("\tdevice_features 0x%x\n", mmio->device_features);
    printf("\tguest_page_size %u\n", mmio->guest_page_size);
    printf("\tqnum %u\n", mmio->queue_num);
    printf("\tqnum_max %u\n", mmio->queue_num_max);
    printf("\tqnum_align %u\n", mmio->queue_align);
    printf("\tqnum_pfn %u\n", mmio->queue_pfn);
    printf("\tstatus 0x%x\n", mmio->status);
}

static enum handler_return virtio_mmio_irq(void *arg)
{
    struct virtio_device *dev = (struct virtio_device *)arg;
    LTRACEF("dev %p, index %u\n", dev, dev->index);

    uint32_t irq_status = dev->mmio.mmio_config->interrupt_status;
    LTRACEF("status 0x%x\n", irq_status);

    enum handler_return ret = INT_NO_RESCHEDULE;
    if (irq_status & 0x1) { /* used ring update */
        // XXX is this safe?
        dev->mmio.mmio_config->interrupt_ack = 0x1;

        /* cycle through all the active rings */
        for (uint r = 0; r < MAX_VIRTIO_RINGS; r++) {
            if ((dev->active_rings_bitmap & (1<<r)) == 0)
                continue;

            struct vring *ring = &dev->ring[r];
            LTRACEF("ring %u: used flags 0x%hhx idx 0x%hhx last_used %u\n",
                    r, ring->used->flags, ring->used->idx, ring->last_used);

            uint cur_idx = ring->used->idx;
            for (uint i = ring->last_used; i != (cur_idx & ring->num_mask); i = (i + 1) & ring->num_mask) {
                LTRACEF("looking at idx %u\n", i);

                // process chain
                struct vring_used_elem *used_elem = &ring->used->ring[i];
                LTRACEF("id %u, len %u\n", used_elem->id, used_elem->len);

                DEBUG_ASSERT(dev->irq_driver_callback);
                ret |= dev->irq_driver_callback(dev, r, used_elem);

                ring->last_used = (ring->last_used + 1) & ring->num_mask;
            }
        }
    }
    if (irq_status & 0x2) { /* config change */
        dev->mmio.mmio_config->interrupt_ack = 0x2;

        if (dev->config_change_callback) {
            ret |= dev->config_change_callback(dev);
        }
    }

    LTRACEF("exiting irq\n");

    return ret;
}

int virtio_mmio_detect(void *ptr, uint count, const uint irqs[])
{
    LTRACEF("ptr %p, count %u\n", ptr, count);

    DEBUG_ASSERT(ptr);
    DEBUG_ASSERT(irqs);
    DEBUG_ASSERT(!virtio_mmio_devices);

    /* allocate an array big enough to hold a list of devices */
    virtio_mmio_devices = calloc(count, sizeof(struct virtio_device));
    if (!virtio_mmio_devices)
        return ERR_NO_MEMORY;

    int found = 0;
    for (uint i = 0; i < count; i++) {
        volatile struct virtio_mmio_config *mmio = (struct virtio_mmio_config *)((uint8_t *)ptr + i * 0x200);
        struct virtio_device *dev = &virtio_mmio_devices[i];

        dev->index = i;
        dev->irq = irqs[i];

        mask_interrupt(irqs[i]);
        register_int_handler(irqs[i], &virtio_mmio_irq, (void *)dev);

        LTRACEF("looking at magic 0x%x version 0x%x did 0x%x vid 0x%x\n",
                mmio->magic, mmio->version, mmio->device_id, mmio->vendor_id);

        if ((mmio->magic != VIRTIO_MMIO_MAGIC) ||
            (mmio->device_id == VIRTIO_DEV_ID_INVALID))
            continue;

#if LOCAL_TRACE
        dump_mmio_config(mmio);
#endif
        const virtio_dev_class_t *class;
        for (class = __start_virtio_classes; class != __stop_virtio_classes; class++) {
            if (mmio->device_id != class->device_id)
                continue;

            LTRACEF("found %s device\n", class->name);
            dev->type = VIO_MMIO;
            dev->mmio.mmio_config = mmio;
            dev->mmio.config_ptr = (void *)mmio->config;

            status_t err = class->init_fn(dev);
            if (err >= 0) {
                // good device
                dev->valid = true;

                if (dev->irq_driver_callback)
                    unmask_interrupt(dev->irq);

                if (class->startup_fn)
                    class->startup_fn(dev);
            } else {
                LTRACEF("Failed to initialize VirtIO MMIO device id %u at position %u (err = %d)\n",
                        mmio->device_id, i, err);

                // indicate to the device that something went fatally wrong on the driver side.
                dev->mmio.mmio_config->status |= VIRTIO_STATUS_FAILED;
            }

            break;
        }

        if (class == __stop_virtio_classes) {
            TRACEF("Unrecognized VirtIO MMIO device id %u discovered at position %u\n",
                    mmio->device_id, i);
        }

        if (dev->valid)
            found++;
    }

    return found;
}

static void virtio_mmio_init(uint level)
{
    /* call all the class driver module init hooks */
    const virtio_dev_class_t *class;
    for (class = __start_virtio_classes; class != __stop_virtio_classes; class++) {
        if (class->module_init_fn)
            class->module_init_fn();
    }
}

// XXX figure out why this fixes stuff
//LK_INIT_HOOK(virtio_mmio, &virtio_mmio_init, LK_INIT_LEVEL_THREADING);


