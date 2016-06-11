#include <dev/virtio.h>

#include <list.h>
#include <trace.h>
#include <err.h>
#include <stdlib.h>
#include <string.h>
#include <pow2.h>
#include <lk/init.h>
#include <dev/interrupt.h>

#include "virtio_priv.h"

#define LOCAL_TRACE 1

#if WITH_DEV_PCIE

#include <dev/pcie.h>

#if 0
static void dump_mmio_config(const volatile struct virtio_mmio_config *mmio)
{
    printf("mmio at %p\n", mmio);
    printf("\tmagic 0x%x\n", mmio->magic);
    printf("\tversion 0x%x\n", mmio->version);
    printf("\tdevice_id 0x%x\n", mmio->device_id);
    printf("\tvendor_id 0x%x\n", mmio->vendor_id);
    printf("\thost_features 0x%x\n", mmio->host_features);
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

    uint32_t irq_status = dev->mmio_config->interrupt_status;
    LTRACEF("status 0x%x\n", irq_status);

    enum handler_return ret = INT_NO_RESCHEDULE;
    if (irq_status & 0x1) { /* used ring update */
        // XXX is this safe?
        dev->mmio_config->interrupt_ack = 0x1;

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
        dev->mmio_config->interrupt_ack = 0x2;

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
            dev->mmio_config = mmio;
            dev->config_ptr = (void *)mmio->config;

            status_t err = class->init_fn(dev, mmio->host_features);
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
                dev->mmio_config->status |= VIRTIO_STATUS_FAILED;
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
#endif

pcie_irq_handler_retval_t virtio_pci_irq_handler(struct pcie_common_state* pci_device,
                                                    uint  irq_id,
                                                    void* ctx)
{
    struct virtio_device *dev = (struct virtio_device *)ctx;

    uint8_t irq_status = pcie_io_read8(dev->pci.pci_control_bar, VIRTIO_PCI_ISR_STATUS);
    LTRACEF("irq_status %u\n", irq_status);

    enum handler_return ret = INT_NO_RESCHEDULE;
    if (irq_status & 0x1) { /* used ring update */
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
        if (dev->config_change_callback) {
            ret |= dev->config_change_callback(dev);
        }
    }

    LTRACEF("exiting irq\n");

    return ret;
}


status_t virtio_add_pci_device(struct virtio_device **_dev, struct pcie_device_state* pci_device)
{
    status_t err = NO_ERROR;

    DEBUG_ASSERT(_dev);
    DEBUG_ASSERT(pci_device);

    struct virtio_device *dev = calloc(1, sizeof(struct virtio_device));
    if (!dev)
        return ERR_NO_MEMORY;

    dev->type = VIO_PCI;

    pcie_enable_pio(pci_device, true);

    // get a pointer to BAR0
    dev->pci.pci_control_bar = pcie_get_bar_info(&pci_device->common, 0);
    if (!dev->pci.pci_control_bar) {
        err = ERR_NOT_FOUND;
        goto err;
    }

    if (dev->pci.pci_control_bar->is_mmio) {
        TRACEF("virtio pci bar0 is mmio, failing\n");
        err = ERR_NOT_FOUND;
        goto err;
    }

    // check the length on queue 0, which is set by the host for legacy pci interfaces
    pcie_io_write16(dev->pci.pci_control_bar, VIRTIO_PCI_QUEUE_SELECT, 0);
    uint16_t len = pcie_io_read16(dev->pci.pci_control_bar, VIRTIO_PCI_QUEUE_SIZE);
    LTRACEF("queue 0 len %u\n", len);

    if (len != 128) {
        TRACEF("legacy pci queue 0 length is not 128, it's %u\n", len);
        err = ERR_NOT_FOUND;
        goto err;
    }

    dev->pci.pci_state = pci_device;
    dev->valid = true;

    *_dev = dev;

    return NO_ERROR;

err:
    free(dev);
    return err;
}

status_t virtio_pci_copy_device_config(struct virtio_device *dev, void *_buf, size_t len)
{
    if (dev->type != VIO_PCI)
        return ERR_INVALID_ARGS;

    // XXX handle MSI vs noMSI
    size_t offset = VIRTIO_PCI_CONFIG_OFFSET_NOMSI;

    uint8_t *buf = (uint8_t *)_buf;
    for (size_t i = 0; i < len; i++) {
        buf[i] = pcie_io_read8(dev->pci.pci_control_bar, offset + i);
    }

    return NO_ERROR;
}

status_t virtio_pci_allocate_irq(struct virtio_device *dev)
{
    LTRACE_ENTRY;

    if (dev->type != VIO_PCI)
        return ERR_INVALID_ARGS;

    status_t ret = pcie_set_irq_mode(&dev->pci.pci_state->common,
                            PCIE_IRQ_MODE_MSI,
                            1,
                            PCIE_IRQ_SHARE_MODE_EXCLUSIVE);
    LTRACEF("pcie_set_irq_mode msi allocate returns %d\n", ret);

    return NO_ERROR;
}

static void virtio_pci_init(uint level)
{
}

LK_INIT_HOOK(virtio_pci, &virtio_pci_init, LK_INIT_LEVEL_THREADING);

#endif // WITH_DEV_PCIE


