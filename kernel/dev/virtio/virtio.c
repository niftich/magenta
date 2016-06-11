// Copyright 2016 The Fuchsia Authors
// Copyright (c) 2014-2015 Travis Geiselbrecht
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

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
#include <kernel/thread.h>
#include <kernel/vm.h>
#include <dev/interrupt.h>

#include "virtio_priv.h"

#if WITH_DEV_PCIE
#include <dev/pcie.h>
#endif

#define LOCAL_TRACE 1

void virtio_dump_desc(const struct vring_desc *desc)
{
    printf("vring descriptor %p\n", desc);
    printf("\taddr  0x%llx\n", desc->addr);
    printf("\tlen   0x%x\n", desc->len);
    printf("\tflags 0x%hhx\n", desc->flags);
    printf("\tnext  0x%hhx\n", desc->next);
}

void virtio_free_desc(struct virtio_device *dev, uint ring_index, uint16_t desc_index)
{
    LTRACEF("dev %p ring %u index %u free_count %u\n", dev, ring_index, desc_index,
            dev->ring[ring_index].free_count);
    dev->ring[ring_index].desc[desc_index].next = dev->ring[ring_index].free_list;
    dev->ring[ring_index].free_list = desc_index;
    dev->ring[ring_index].free_count++;
}

void virtio_free_desc_chain(struct virtio_device *dev, uint ring_index, uint16_t chain_head)
{
    struct vring_desc* desc = virtio_desc_index_to_desc(dev, ring_index, chain_head);

    while (desc->flags & VRING_DESC_F_NEXT) {
        uint16_t next = desc->next;
        virtio_free_desc(dev, ring_index, chain_head);
        chain_head = next;
        desc = virtio_desc_index_to_desc(dev, ring_index, chain_head);
    }

    virtio_free_desc(dev, ring_index, chain_head);
}

uint16_t virtio_alloc_desc(struct virtio_device *dev, uint ring_index)
{
    if (dev->ring[ring_index].free_count == 0)
        return 0xffff;

    DEBUG_ASSERT(dev->ring[ring_index].free_list != 0xffff);

    uint16_t i = dev->ring[ring_index].free_list;
    struct vring_desc *desc = &dev->ring[ring_index].desc[i];
    dev->ring[ring_index].free_list = desc->next;

    dev->ring[ring_index].free_count--;

    return i;
}

struct vring_desc *virtio_alloc_desc_chain(struct virtio_device *dev, uint ring_index,
        size_t count, uint16_t *start_index)
{
    if (dev->ring[ring_index].free_count < count)
        return NULL;

    /* start popping entries off the chain */
    struct vring_desc *last = 0;
    uint16_t last_index = 0;
    while (count > 0) {
        uint16_t i = dev->ring[ring_index].free_list;
        struct vring_desc *desc = &dev->ring[ring_index].desc[i];

        dev->ring[ring_index].free_list = desc->next;
        dev->ring[ring_index].free_count--;

        if (last) {
            desc->flags = VRING_DESC_F_NEXT;
            desc->next = last_index;
        } else {
            // first one
            desc->flags = 0;
            desc->next = 0;
        }
        last = desc;
        last_index = i;
        count--;
    }

    if (start_index)
        *start_index = last_index;

    return last;
}

void virtio_submit_chain(struct virtio_device *dev, uint ring_index, uint16_t desc_index)
{
    LTRACEF("dev %p, ring %u, desc %u\n", dev, ring_index, desc_index);

    /* add the chain to the available list */
    struct vring_avail *avail = dev->ring[ring_index].avail;

    avail->ring[avail->idx & dev->ring[ring_index].num_mask] = desc_index;
    mb();
    avail->idx++;

#if LOCAL_TRACE
    hexdump(avail, 16);
#endif
}

void virtio_kick(struct virtio_device *dev, uint ring_index)
{
    LTRACEF("dev %p, ring %u\n", dev, ring_index);

    if (dev->type == VIO_MMIO) {
        dev->mmio.mmio_config->queue_notify = ring_index;
        mb();
#if WITH_DEV_PCIE
    } else if (dev->type == VIO_PCI) {
        pcie_io_write16(dev->pci.pci_control_bar, VIRTIO_PCI_QUEUE_NOTIFY, ring_index);
#endif
    }
}

status_t virtio_alloc_ring(struct virtio_device *dev, uint index, uint16_t len)
{
    LTRACEF("dev %p, index %u, len %u\n", dev, index, len);

    DEBUG_ASSERT(dev);
    DEBUG_ASSERT(len > 0 && ispow2(len));
    DEBUG_ASSERT(index < MAX_VIRTIO_RINGS);

    if (len == 0 || !ispow2(len))
        return ERR_INVALID_ARGS;

    struct vring *ring = &dev->ring[index];

    /* allocate a ring */
    size_t size = vring_size(len, PAGE_SIZE);
    LTRACEF("need %zu bytes\n", size);

#if WITH_KERNEL_VM
    void *vptr;
    status_t err = vmm_alloc_contiguous(vmm_get_kernel_aspace(), "virtio_ring", size, &vptr,
            0, VMM_FLAG_COMMIT, ARCH_MMU_FLAG_UNCACHED_DEVICE);
    if (err < 0)
        return ERR_NO_MEMORY;

    LTRACEF("allocated virtio_ring at va %p\n", vptr);

    /* compute the physical address */
    paddr_t pa;
    pa = vaddr_to_paddr(vptr);
    if (pa == 0) {
        return ERR_NO_MEMORY;
    }

    LTRACEF("virtio_ring at pa 0x%lx\n", pa);
#else
    void *vptr = memalign(PAGE_SIZE, size);
    if (!vptr)
        return ERR_NO_MEMORY;

    LTRACEF("ptr %p\n", vptr);
    memset(vptr, 0, size);

    /* compute the physical address */
    paddr_t pa = (paddr_t)vptr;
#endif

    /* initialize the ring */
    vring_init(ring, len, vptr, PAGE_SIZE);
    dev->ring[index].free_list = 0xffff;
    dev->ring[index].free_count = 0;

    /* add all the descriptors to the free list */
    for (uint i = 0; i < len; i++) {
        virtio_free_desc(dev, index, i);
    }

    /* register the ring with the device */
    if (dev->type == VIO_MMIO) {
        dev->mmio.mmio_config->guest_page_size = PAGE_SIZE;
        dev->mmio.mmio_config->queue_sel = index;
        dev->mmio.mmio_config->queue_num = len;
        dev->mmio.mmio_config->queue_align = PAGE_SIZE;
        dev->mmio.mmio_config->queue_pfn = pa / PAGE_SIZE;
#if WITH_DEV_PCIE
    } else if (dev->type == VIO_PCI) {
        pcie_io_write16(dev->pci.pci_control_bar, VIRTIO_PCI_QUEUE_SELECT, index);
        pcie_io_write16(dev->pci.pci_control_bar, VIRTIO_PCI_QUEUE_SIZE, len);
        pcie_io_write32(dev->pci.pci_control_bar, VIRTIO_PCI_QUEUE_PFN, pa / PAGE_SIZE);
#endif
    }

    /* mark the ring active */
    dev->active_rings_bitmap |= (1 << index);

    return NO_ERROR;
}

void virtio_reset_device(struct virtio_device *dev)
{
    if (dev->type == VIO_MMIO)
        dev->mmio.mmio_config->status = 0;
#if WITH_DEV_PCIE
    else if (dev->type == VIO_PCI)
        pcie_io_write8(dev->pci.pci_control_bar, VIRTIO_PCI_DEVICE_STATUS, 0);
#endif
}

void virtio_status_acknowledge_driver(struct virtio_device *dev)
{
    if (dev->type == VIO_MMIO)
        dev->mmio.mmio_config->status |= VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER;
#if WITH_DEV_PCIE
    else if (dev->type == VIO_PCI) {
        uint8_t val = pcie_io_read8(dev->pci.pci_control_bar, VIRTIO_PCI_DEVICE_STATUS);
        val |= VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER;
        pcie_io_write8(dev->pci.pci_control_bar, VIRTIO_PCI_DEVICE_STATUS, val);
    }
#endif
}

void virtio_status_driver_ok(struct virtio_device *dev)
{
    if (dev->type == VIO_MMIO)
        dev->mmio.mmio_config->status |= VIRTIO_STATUS_DRIVER_OK;
#if WITH_DEV_PCIE
    else if (dev->type == VIO_PCI) {
        uint8_t val = pcie_io_read8(dev->pci.pci_control_bar, VIRTIO_PCI_DEVICE_STATUS);
        val |= VIRTIO_STATUS_DRIVER_OK;
        pcie_io_write8(dev->pci.pci_control_bar, VIRTIO_PCI_DEVICE_STATUS, val);
    }
#endif
}

void virtio_init(uint level)
{
}

LK_INIT_HOOK(virtio, &virtio_init, LK_INIT_LEVEL_THREADING);

