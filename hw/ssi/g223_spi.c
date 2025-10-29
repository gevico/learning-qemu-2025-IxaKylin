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

#ifndef DEBUG_G223_SPI
#define DEBUG_G223_SPI 0
#endif

#define DPRINTF(fmt, args...) \
    do { \
        if (DEBUG_G223_SPI) { \
            fprintf(stderr, "[%s]%s: " fmt , TYPE_G223_SPI, \
                                             __func__, ##args); \
        } \
    } while (0)

static bool g223_spi_is_enabled(G223SPIState *s)
{
    return s->cr1 & G233_SPI_CR1_SPE;
}

static bool g223_spi_is_busy(G223SPIState *s)
{
    return s->sr & G233_SPI_SR_BUSY;
}

// static bool g223_spi_is_master(G223SPIState *s)
// {
//     return s->cr1 & G233_SPI_CR1_MSTR;
// }

static bool g223_spi_channel_enabled(G223SPIState *s)
{
    uint8_t cs_bits = (s->csctrl >> 4) & 0xf;  /* Bits 4:7 - chip select bits */
    uint8_t channel_bits = s->csctrl & 0xf;    /* Bits 0:3 - channel enable bits */
    
    /* Check if corresponding bits are both set to 1 */
    return (cs_bits & channel_bits) != 0;
}

static void g223_spi_update_irq(G223SPIState *s) {
    int level;
    
    if (fifo8_is_empty(&s->rx_fifo)) {
        s->sr &= ~G233_SPI_SR_RXNE;
    } else {
        s->sr |= G233_SPI_SR_RXNE;
    }

    if (fifo8_is_full(&s->tx_fifo)) {
        s->sr &= ~G233_SPI_SR_TXE;
    } else {
        s->sr |= G233_SPI_SR_TXE;
    }

    //G233_SPI_SR_OVERRUN and G233_SPI_SR_UNDERRUN not need by auto cleared

    if (s->cr2 & G233_SPI_CR2_SSOE) {
        level = ( (s->sr & G233_SPI_SR_RXNE)      && (s->cr2 & G233_SPI_CR2_RXNEIE) ) ||
                ( (s->sr & G233_SPI_SR_TXE)       && (s->cr2 & G233_SPI_CR2_TXEIE)  ) || 
                ( (s->sr & G233_SPI_SR_UNDERRUN ) && (s->cr2 & G233_SPI_CR2_ERRIE)  ) ||
                ( (s->sr & G233_SPI_SR_OVERRUN )  && (s->cr2 & G233_SPI_CR2_ERRIE)  ) ? 1 : 0;
    } else {
        level = 0;
    }

    qemu_set_irq(s->irq, level);

    DPRINTF("IRQ level is %d\n", level);
}

static void g223_spi_txfifo_reset(G223SPIState *s)
{
    fifo8_reset(&s->tx_fifo);
    s->sr |= G233_SPI_SR_TXE;
}

static void g223_spi_rxfifo_reset(G223SPIState *s)
{
    fifo8_reset(&s->rx_fifo);
    s->sr &= ~G233_SPI_SR_RXNE;
}

static void g223_spi_flush_tx_fifo(G223SPIState *s)
{
    uint8_t tx_byte, rx_byte;

    while (!fifo8_is_empty(&s->tx_fifo) && !fifo8_is_full(&s->rx_fifo)) {
        tx_byte = fifo8_pop(&s->tx_fifo);
        rx_byte = ssi_transfer(s->bus, tx_byte);
        fifo8_push(&s->rx_fifo, rx_byte);
    }
}

// static void g223_spi_flush_txfifo(G223SPIState *s)
// {
//     DPRINTF("Begin: TX Fifo Size = %d, RX Fifo Size = %d\n",
//         fifo8_num_used(&s->tx_fifo), fifo8_num_used(&s->rx_fifo));

// }

static uint64_t g223_spi_read(void *opaque, hwaddr addr, unsigned size) {
    G223SPIState *s = opaque;
    uint32_t readval = 0;

    switch (addr) {
        case G233_SPI_CR1:
            readval = s->cr1 & 0xffffffff;
            break;
        case G233_SPI_CR2:
            readval = s->cr2 & 0xffffffff;
            break;
        case G233_SPI_SR:
            readval = s->sr & 0xffffffff;
            break;
        case G233_SPI_DR:
            if (g223_spi_is_enabled(s) && g223_spi_channel_enabled(s)) {
                if (!g223_spi_is_busy(s)) {
                    s->sr |= G233_SPI_SR_BUSY;
                    readval = fifo8_pop(&s->rx_fifo);
                    g223_spi_flush_tx_fifo(s);
                    s->sr &= ~G233_SPI_SR_BUSY;
                }
                
            }
            break;
        case G233_SPI_CSCTRL:
            readval = s->csctrl & 0xffffffff;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }
    g223_spi_update_irq(s);
    return readval;
}

static void g223_spi_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    G223SPIState *s = opaque;

    if (g223_spi_is_busy(s)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Cannot write while SPI is busy\n", __func__);
        return;
    }

    // set busy flag
    s->sr |= G233_SPI_SR_BUSY;
    
    switch (addr) {
        case G233_SPI_CR1:
            s->cr1 = (s->cr1 & ~(G233_SPI_CR1_MSTR | G233_SPI_CR1_SPE)) |
                    (value & (G233_SPI_CR1_MSTR | G233_SPI_CR1_SPE));
            break;
        case G233_SPI_CR2:
            s->cr2 = (s->cr2 & ~(G233_SPI_CR2_SSOE | G233_SPI_CR2_ERRIE | 
                                G233_SPI_CR2_RXNEIE | G233_SPI_CR2_TXEIE)) |
                     (value & (G233_SPI_CR2_SSOE | G233_SPI_CR2_ERRIE | 
                              G233_SPI_CR2_RXNEIE | G233_SPI_CR2_TXEIE));
            break;
        case G233_SPI_SR:
            /* Check if OVERRUN or UNDERRUN bits are being cleared from 1 to 0 */
            uint32_t old_sr = s->sr;
            uint32_t new_sr = (old_sr & ~(G233_SPI_SR_OVERRUN | G233_SPI_SR_UNDERRUN)) |
                             (value & (G233_SPI_SR_OVERRUN | G233_SPI_SR_UNDERRUN));
            
            /* If OVERRUN bit is being cleared from 1 to 0, reset TX FIFO */
            if ((old_sr & G233_SPI_SR_OVERRUN) && !(new_sr & G233_SPI_SR_OVERRUN)) {
                g223_spi_txfifo_reset(s);
            }
            
            /* If UNDERRUN bit is being cleared from 1 to 0, reset RX FIFO */
            if ((old_sr & G233_SPI_SR_UNDERRUN) && !(new_sr & G233_SPI_SR_UNDERRUN)) {
                g223_spi_rxfifo_reset(s);
            }
            
            s->sr = new_sr;
            break;
        case G233_SPI_CSCTRL:
            /* Validate CSCTRL register value */
            /* Bits 31:8 must be 0 */
            if (value & 0xffffff00) {
                qemu_log_mask(LOG_GUEST_ERROR,
                            "%s: CSCTRL bits 31:8 must be 0, got 0x%08" PRIx64 "\n",
                            __func__, value);
                break;
            }
            
            /* Bits 4:7 must have exactly one bit set */
            uint8_t cs_bits = (value >> 4) & 0xf;
            if (cs_bits != 0 && (cs_bits & (cs_bits - 1)) != 0) {
                qemu_log_mask(LOG_GUEST_ERROR,
                            "%s: CSCTRL bits 4:7 must have exactly one bit set, got 0x%x\n",
                            __func__, cs_bits);
                break;
            }
            
            s->csctrl = value & 0xffffffff;
            break;
        case G233_SPI_DR:
            if (g223_spi_is_enabled(s) && g223_spi_channel_enabled(s)) {
                if (fifo8_is_full(&s->tx_fifo)) {
                    // TX FIFO full, set OVERRUN flag
                    s->sr |= G233_SPI_SR_OVERRUN;
                } else {
                    fifo8_push(&s->tx_fifo, (uint32_t)value);
                    
                }
                g223_spi_flush_tx_fifo(s);
            }

            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }

    //clear busy flag
    s->sr &= ~G233_SPI_SR_BUSY;
    g223_spi_update_irq(s);
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
    dc->realize = g223_spi_realize;
    dc->vmsd = &vmstate_g223_spi;
}

static const TypeInfo g223_spi_info = {
    .name           = TYPE_G223_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(G223SPIState),
    .class_init     = g223_spi_class_init,
};

static void g223_spi_register_types(void)
{
    type_register_static(&g223_spi_info);
}

type_init(g223_spi_register_types)
