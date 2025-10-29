/*
 * BCM2835 SPI Master Controller
 *
 * Copyright (c) 2024 Rayhan Faizel <rayhan.faizel@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/fifo8.h"
#include "hw/ssi/g223_spi.h"
#include "hw/irq.h"
#include "migration/vmstate.h"

static void g223_spi_update_int(G223SPIState *s)
{
    int do_interrupt = 0;
    //todo
}

static uint64_t g223_spi_read(void *opaque, hwaddr addr, unsigned size) {
    //todo
}

static void g223_spi_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    //todo
}

static const MemoryRegionOps g223_spi_ops = {
    .read = g223_spi_read,
    .write = g223_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void g223_spi_realize(DeviceState *dev, Error **errp) {
    G223SPIState *s = G223_SPI(dev);
    s->bus = ssi_create_bus(dev, "spi");

    memory_region_init_io(&s->iomem, OBJECT(dev), &g223_spi_ops, s,
                          TYPE_G223_SPI, 0x14); // load how much register 5 x 32bit = 0x14
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    fifo8_create(&s->tx_fifo, FIFO_SIZE);
    fifo8_create(&s->rx_fifo, FIFO_SIZE);
}

static void g223_spi_reset(DeviceState *dev) {
    G223SPIState *s = G223_SPI(dev);
    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);

    s->cr1 = 0;
    s->cr2 = 0;
    s->sr = 0x00000002;
    s->dr = 0x0000000C;
    s->csctrl = 0;

}

static const VMStateDescription vmstate_g223_spi = {
    .name = TYPE_G223_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_FIFO8(tx_fifo, G223SPIState),
        VMSTATE_FIFO8(rx_fifo, G223SPIState),
        VMSTATE_UINT32(cr1, G223SPIState),
        VMSTATE_UINT32(cr2, G223SPIState),
        VMSTATE_UINT32(sr, G223SPIState),
        VMSTATE_UINT32(dr, G223SPIState),
        VMSTATE_UINT32(csctrl, G223SPIState),
        VMSTATE_END_OF_LIST()
    }
};

static void g223_spi_class_init(ObjectClass *klass, const void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, g223_spi_reset);
    dc->vmsd = &vmstate_g223_spi;
}

static const TypeInfo g223_spi_info = {
    .name           = TYPE_G223_SPI,
    .parent         = TYPE_SIFIVE_SPI,
    .instance_size  = sizeof(G223SPIState),
    .class_init     = NULL,
};

static void g223_spi_register_types(void)
{
    type_register_static(&g223_spi_info);
}

type_init(g223_spi_register_types)