/*
 * QEMU educational PCI device
 *
 * Copyright (c) 2012-2015 Jiri Slaby
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/hw.h"
#include "hw/pci/msi.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "qapi/visitor.h"

#define TYPE_PCI_OVL_DEVICE "ovl"
typedef struct EduState EduState;
DECLARE_INSTANCE_CHECKER(EduState, OVL,
                         TYPE_PCI_OVL_DEVICE)

#define FACT_IRQ        0x00000001
#define DMA_IRQ         0x00000100

#define DMA_START       0x40000
#define DMA_SIZE        4096

struct EduState {
    PCIDevice pdev;
    MemoryRegion mmio;

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stopping;

    uint32_t addr4;
    uint32_t fact;
#define OVL_STATUS_COMPUTING    0x01
#define OVL_STATUS_IRQFACT      0x80
    uint32_t status;

    uint32_t irq_status;

#define OVL_DMA_RUN             0x1
#define OVL_DMA_DIR(cmd)        (((cmd) & 0x2) >> 1)
# define OVL_DMA_FROM_PCI       0
# define OVL_DMA_TO_PCI         1
#define OVL_DMA_IRQ             0x4
    struct dma_state {
        dma_addr_t src;
        dma_addr_t dst;
        dma_addr_t cnt;
        dma_addr_t cmd;
    } dma;
    QEMUTimer dma_timer;
    char dma_buf[DMA_SIZE];
    uint64_t dma_mask;
};

static bool ovl_msi_enabled(EduState *ovl)
{
    return msi_enabled(&ovl->pdev);
}

static void ovl_raise_irq(EduState *ovl, uint32_t val)
{
    ovl->irq_status |= val;
    if (ovl->irq_status) {
        if (ovl_msi_enabled(ovl)) {
            msi_notify(&ovl->pdev, 0);
        } else {
            pci_set_irq(&ovl->pdev, 1);
        }
    }
}

static void ovl_lower_irq(EduState *ovl, uint32_t val)
{
    ovl->irq_status &= ~val;

    if (!ovl->irq_status && !ovl_msi_enabled(ovl)) {
        pci_set_irq(&ovl->pdev, 0);
    }
}

static bool within(uint64_t addr, uint64_t start, uint64_t end)
{
    return start <= addr && addr < end;
}

static void ovl_check_range(uint64_t addr, uint64_t size1, uint64_t start,
                uint64_t size2)
{
    uint64_t end1 = addr + size1;
    uint64_t end2 = start + size2;

    if (within(addr, start, end2) &&
            end1 > addr && within(end1, start, end2)) {
        return;
    }

    hw_error("OVL: DMA range 0x%016"PRIx64"-0x%016"PRIx64
             " out of bounds (0x%016"PRIx64"-0x%016"PRIx64")!",
            addr, end1 - 1, start, end2 - 1);
}

static dma_addr_t ovl_clamp_addr(const EduState *ovl, dma_addr_t addr)
{
    dma_addr_t res = addr & ovl->dma_mask;

    if (addr != res) {
        printf("OVL: clamping DMA %#.16"PRIx64" to %#.16"PRIx64"!\n", addr, res);
    }

    return res;
}

static void ovl_dma_timer(void *opaque)
{
    EduState *ovl = opaque;
    bool raise_irq = false;

    if (!(ovl->dma.cmd & OVL_DMA_RUN)) {
        return;
    }

    if (OVL_DMA_DIR(ovl->dma.cmd) == OVL_DMA_FROM_PCI) {
        uint64_t dst = ovl->dma.dst;
        ovl_check_range(dst, ovl->dma.cnt, DMA_START, DMA_SIZE);
        dst -= DMA_START;
        pci_dma_read(&ovl->pdev, ovl_clamp_addr(ovl, ovl->dma.src),
                ovl->dma_buf + dst, ovl->dma.cnt);
    } else {
        uint64_t src = ovl->dma.src;
        ovl_check_range(src, ovl->dma.cnt, DMA_START, DMA_SIZE);
        src -= DMA_START;
        pci_dma_write(&ovl->pdev, ovl_clamp_addr(ovl, ovl->dma.dst),
                ovl->dma_buf + src, ovl->dma.cnt);
    }

    ovl->dma.cmd &= ~OVL_DMA_RUN;
    if (ovl->dma.cmd & OVL_DMA_IRQ) {
        raise_irq = true;
    }

    if (raise_irq) {
        ovl_raise_irq(ovl, DMA_IRQ);
    }
}

static void dma_rw(EduState *ovl, bool write, dma_addr_t *val, dma_addr_t *dma,
                bool timer)
{
    if (write && (ovl->dma.cmd & OVL_DMA_RUN)) {
        return;
    }

    if (write) {
        *dma = *val;
    } else {
        *val = *dma;
    }

    if (timer) {
        timer_mod(&ovl->dma_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
    }
}

static uint64_t ovl_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    EduState *ovl = opaque;
    uint64_t val = ~0ULL;

    if (addr < 0x80 && size != 4) {
        return val;
    }

    if (addr >= 0x80 && size != 4 && size != 8) {
        return val;
    }

    switch (addr) {
    case 0x00:
        val = 0x010000;
        break;
    case 0x04:
        val = ovl->addr4;
        break;
    case 0x08:
        qemu_mutex_lock(&ovl->thr_mutex);
        val = ovl->fact;
        qemu_mutex_unlock(&ovl->thr_mutex);
        break;
    case 0x20:
        val = qatomic_read(&ovl->status);
        break;
    case 0x24:
        val = ovl->irq_status;
        break;
    case 0x80:
        dma_rw(ovl, false, &val, &ovl->dma.src, false);
        break;
    case 0x88:
        dma_rw(ovl, false, &val, &ovl->dma.dst, false);
        break;
    case 0x90:
        dma_rw(ovl, false, &val, &ovl->dma.cnt, false);
        break;
    case 0x98:
        dma_rw(ovl, false, &val, &ovl->dma.cmd, false);
        break;
    }

    return val;
}

static void ovl_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    EduState *ovl = opaque;

    if (addr < 0x80 && size != 4) {
        return;
    }

    if (addr >= 0x80 && size != 4 && size != 8) {
        return;
    }

    switch (addr) {
    case 0x04:
        ovl->addr4 = ~val;
        break;
    case 0x08:
        if (qatomic_read(&ovl->status) & OVL_STATUS_COMPUTING) {
            break;
        }
        /* OVL_STATUS_COMPUTING cannot go 0->1 concurrently, because it is only
         * set in this function and it is under the iothread mutex.
         */
        qemu_mutex_lock(&ovl->thr_mutex);
        ovl->fact = val;
        qatomic_or(&ovl->status, OVL_STATUS_COMPUTING);
        qemu_cond_signal(&ovl->thr_cond);
        qemu_mutex_unlock(&ovl->thr_mutex);
        break;
    case 0x20:
        if (val & OVL_STATUS_IRQFACT) {
            qatomic_or(&ovl->status, OVL_STATUS_IRQFACT);
        } else {
            qatomic_and(&ovl->status, ~OVL_STATUS_IRQFACT);
        }
        break;
    case 0x60:
        ovl_raise_irq(ovl, val);
        break;
    case 0x64:
        ovl_lower_irq(ovl, val);
        break;
    case 0x80:
        dma_rw(ovl, true, &val, &ovl->dma.src, false);
        break;
    case 0x88:
        dma_rw(ovl, true, &val, &ovl->dma.dst, false);
        break;
    case 0x90:
        dma_rw(ovl, true, &val, &ovl->dma.cnt, false);
        break;
    case 0x98:
        if (!(val & OVL_DMA_RUN)) {
            break;
        }
        dma_rw(ovl, true, &val, &ovl->dma.cmd, true);
        break;
    }
}

static const MemoryRegionOps ovl_mmio_ops = {
    .read = ovl_mmio_read,
    .write = ovl_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },

};

/*
 * We purposely use a thread, so that users are forced to wait for the status
 * register.
 */
static void *ovl_fact_thread(void *opaque)
{
    EduState *ovl = opaque;

    while (1) {
        uint32_t val, ret = 1;

        qemu_mutex_lock(&ovl->thr_mutex);
        while ((qatomic_read(&ovl->status) & OVL_STATUS_COMPUTING) == 0 &&
                        !ovl->stopping) {
            qemu_cond_wait(&ovl->thr_cond, &ovl->thr_mutex);
        }

        if (ovl->stopping) {
            qemu_mutex_unlock(&ovl->thr_mutex);
            break;
        }

        val = ovl->fact;
        qemu_mutex_unlock(&ovl->thr_mutex);

        while (val > 0) {
            ret *= val--;
        }

        /*
         * We should sleep for a random period here, so that students are
         * forced to check the status properly.
         */

        qemu_mutex_lock(&ovl->thr_mutex);
        ovl->fact = ret;
        qemu_mutex_unlock(&ovl->thr_mutex);
        qatomic_and(&ovl->status, ~OVL_STATUS_COMPUTING);

        if (qatomic_read(&ovl->status) & OVL_STATUS_IRQFACT) {
            qemu_mutex_lock_iothread();
            ovl_raise_irq(ovl, FACT_IRQ);
            qemu_mutex_unlock_iothread();
        }
    }

    return NULL;
}

static void pci_ovl_realize(PCIDevice *pdev, Error **errp)
{
    EduState *ovl = OVL(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }

    timer_init_ms(&ovl->dma_timer, QEMU_CLOCK_VIRTUAL, ovl_dma_timer, ovl);

    qemu_mutex_init(&ovl->thr_mutex);
    qemu_cond_init(&ovl->thr_cond);
    qemu_thread_create(&ovl->thread, "ovl", ovl_fact_thread,
                       ovl, QEMU_THREAD_JOINABLE);

    memory_region_init_io(&ovl->mmio, OBJECT(ovl), &ovl_mmio_ops, ovl,
                    "ovl-mmio", 1 * MiB);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &ovl->mmio);
}

static void pci_ovl_uninit(PCIDevice *pdev)
{
    EduState *ovl = OVL(pdev);

    qemu_mutex_lock(&ovl->thr_mutex);
    ovl->stopping = true;
    qemu_mutex_unlock(&ovl->thr_mutex);
    qemu_cond_signal(&ovl->thr_cond);
    qemu_thread_join(&ovl->thread);

    qemu_cond_destroy(&ovl->thr_cond);
    qemu_mutex_destroy(&ovl->thr_mutex);

    timer_del(&ovl->dma_timer);
    msi_uninit(pdev);
}

static void ovl_instance_init(Object *obj)
{
    EduState *ovl = OVL(obj);

    ovl->dma_mask = (1UL << 28) - 1;
    object_property_add_uint64_ptr(obj, "dma_mask",
                                   &ovl->dma_mask, OBJ_PROP_FLAG_READWRITE);
}

static void ovl_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_ovl_realize;
    k->exit = pci_ovl_uninit;
    k->vendor_id = 0x1414;
    k->device_id = 0xb9;
    k->revision = 0x0;
    k->class_id = PCI_CLASS_OTHERS;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static void pci_ovl_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo ovl_info = {
        .name          = TYPE_PCI_OVL_DEVICE,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(EduState),
        .instance_init = ovl_instance_init,
        .class_init    = ovl_class_init,
        .interfaces = interfaces,
    };

    type_register_static(&ovl_info);
}
type_init(pci_ovl_register_types)
