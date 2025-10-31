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
#include "hw/ssi/g233_spi.h"
#include "hw/irq.h"
#include "migration/vmstate.h"

#ifndef DEBUG_G233_SPI
#define DEBUG_G233_SPI 0
#endif

#define DPRINTF(fmt, args...) \
    do { \
        if (DEBUG_G233_SPI) { \
            fprintf(stderr, "[%s]%s: " fmt , TYPE_G233_SPI, \
                                             __func__, ##args); \
        } \
    } while (0)

static bool g233_spi_is_enabled(G233SPIState *s)
{
    return s->cr1 & G233_SPI_CR1_SPE;
}

// static void g233_spi_set_busy(G233SPIState *s, bool busy)
// {
//     if (busy) {
//         s->sr |= G233_SPI_SR_BUSY;
//     } else {
//         s->sr &= ~G233_SPI_SR_BUSY;
//     }
// }

// static bool g233_spi_is_busy(G233SPIState *s)
// {
//     return s->sr & G233_SPI_SR_BUSY;
// }

// static bool g233_spi_is_master(G233SPIState *s)
// {
//     return s->cr1 & G233_SPI_CR1_MSTR;
// }

// static bool g233_spi_channel_enabled(G233SPIState *s, int channel)
// {
//     /* 检查指定通道是否启用且激活 */
//     if (s->csctrl & (1 << channel)) {
//         /* 通道启用，检查激活状态 */
//         return (s->csctrl & (1 << (channel + 4))) != 0;
//     }
//     /* 通道禁用 */
//     return false;
// }

static inline int get_act_cs(uint32_t csctrl, int cs)
{
    return csctrl & G233_SPI_CS_GET_ACT(csctrl, cs);
}

static void g233_spi_update_cs(G233SPIState* s)
{
    for (int i = 0; i < s->num_cs; i++) {
        if (s->csctrl & G233_SPI_CS_EN(i)) {
            qemu_set_irq(s->cs_lines[i], !get_act_cs(s->csctrl, i));
        } else {
            qemu_irq_raise(s->cs_lines[i]);
        }
    }
}

static void g233_spi_update_irq(G233SPIState *s) {
    int level;

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

static void g233_spi_txfifo_reset(G233SPIState *s)
{
    fifo8_reset(&s->tx_fifo);
    s->sr |= G233_SPI_SR_TXE;
}

static void g233_spi_rxfifo_reset(G233SPIState *s)
{
    fifo8_reset(&s->rx_fifo);
    s->sr &= ~G233_SPI_SR_RXNE;
}

static void g233_spi_flush_tx_fifo(G233SPIState *s)
{
    uint8_t tx_byte, rx_byte;

    while (!fifo8_is_empty(&s->tx_fifo) && !fifo8_is_full(&s->rx_fifo)) {
        tx_byte = fifo8_pop(&s->tx_fifo);
        rx_byte = ssi_transfer(s->bus, tx_byte);
        fifo8_push(&s->rx_fifo, rx_byte);
    }
}

static uint64_t g233_spi_read(void *opaque, hwaddr addr, unsigned size) {
    G233SPIState *s = opaque;
    uint32_t readval = 0;
    //g233_spi_set_busy(s, true);
    switch (addr) {
        case G233_SPI_CR1:
            readval = s->cr1 & 0xffffffff;
            break;
        case G233_SPI_CR2:
            readval = s->cr2 & 0xffffffff;
            break;
        case G233_SPI_SR:
            if (fifo8_is_empty(&s->rx_fifo)) {
                s->sr &= ~G233_SPI_SR_RXNE;  // 接收缓冲区空，清除RXNE标志
            } else {
                s->sr |= G233_SPI_SR_RXNE;   // 接收缓冲区非空，设置RXNE标志
            }

            // TXE: 发送缓冲区空标志
            if (fifo8_is_empty(&s->tx_fifo)) {
                s->sr |= G233_SPI_SR_TXE;    // 发送缓冲区空，设置TXE标志
            } else {
                s->sr &= ~G233_SPI_SR_TXE;   // 发送缓冲区满，清除TXE标志
            }
            readval = s->sr & 0xffffffff;
            break;
        case G233_SPI_DR:
            if (g233_spi_is_enabled(s)) {
                if (fifo8_is_empty(&s->rx_fifo)) {
                    // RX FIFO empty, set UNDERRUN flag
                    s->sr |= G233_SPI_SR_UNDERRUN;
                    readval = 0; // 返回默认值
                } else {
                    readval = fifo8_pop(&s->rx_fifo);
                }
                g233_spi_flush_tx_fifo(s);
                if (fifo8_is_empty(&s->rx_fifo)) {
                    s->sr |= G233_SPI_SR_RXNE;
                } else {
                    s->sr &= ~G233_SPI_SR_RXNE;
                }

                if (fifo8_is_empty(&s->tx_fifo)) {
                    s->sr |= G233_SPI_SR_TXE;
                } else {
                    s->sr &= ~G233_SPI_SR_TXE;
                }
            }
            g233_spi_update_irq(s);
            break;
        case G233_SPI_CSCTRL:
            readval = s->csctrl & 0xffffffff;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }

    //g233_spi_set_busy(s, false);
    return readval;
}

static void g233_spi_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    G233SPIState *s = opaque;
    //g233_spi_set_busy(s, true);
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
            g233_spi_update_irq(s);
            break;
        case G233_SPI_SR:
            /* Check if OVERRUN or UNDERRUN bits are being cleared from 1 to 0 */
            uint32_t old_sr = s->sr;
            uint32_t new_sr = (old_sr & ~(G233_SPI_SR_OVERRUN | G233_SPI_SR_UNDERRUN)) |
                             (value & (G233_SPI_SR_OVERRUN | G233_SPI_SR_UNDERRUN));
            
            /* If OVERRUN bit is being cleared from 1 to 0, reset TX FIFO */
            if ((old_sr & G233_SPI_SR_OVERRUN) && !(new_sr & G233_SPI_SR_OVERRUN)) {
                g233_spi_txfifo_reset(s);
            }
            
            /* If UNDERRUN bit is being cleared from 1 to 0, reset RX FIFO */
            if ((old_sr & G233_SPI_SR_UNDERRUN) && !(new_sr & G233_SPI_SR_UNDERRUN)) {
                g233_spi_rxfifo_reset(s);
            }
            
            s->sr = new_sr;
            g233_spi_update_irq(s);
            break;
        case G233_SPI_CSCTRL: 
            s->csctrl = value & 0xffffffff;
            g233_spi_update_cs(s);
            break;
        case G233_SPI_DR:
            if (g233_spi_is_enabled(s)) {
                if (fifo8_is_full(&s->tx_fifo)) {
                    // TX FIFO full, set OVERRUN flag
                    s->sr |= G233_SPI_SR_OVERRUN;
                } else {
                    fifo8_push(&s->tx_fifo, (uint32_t)value);
                }
                g233_spi_flush_tx_fifo(s);
                if (fifo8_is_empty(&s->rx_fifo)) {    
                    s->sr |= G233_SPI_SR_RXNE;
                } else {
                    s->sr &= ~G233_SPI_SR_RXNE;
                }

                if (fifo8_is_empty(&s->tx_fifo)) {
                    s->sr |= G233_SPI_SR_TXE;
                } else {
                    s->sr &= ~G233_SPI_SR_TXE;
                }
            }
            g233_spi_update_irq(s);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }

    //g233_spi_set_busy(s, false);
    
}

static const MemoryRegionOps g233_spi_ops = {
    .read = g233_spi_read,
    .write = g233_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
    .min_access_size = 4,
    .max_access_size = 4,
    },
};

static void g233_spi_realize(DeviceState *dev, Error **errp) {
    G233SPIState* s = G233_SPI(dev);
    s->bus = ssi_create_bus(dev, "spi");

    memory_region_init_io(&s->iomem, OBJECT(dev), &g233_spi_ops, s, 
                          TYPE_G233_SPI, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->num_cs = G233_CS_NUM;
    for (int i = 0; i < 4; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->cs_lines[i]);
    }

    fifo8_create(&s->tx_fifo, 1);
    fifo8_create(&s->rx_fifo, 1);
}

static void g233_spi_reset(DeviceState *dev) {
    G233SPIState *s = G233_SPI(dev);
    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);

    s->cr1 = 0;
    s->cr2 = 0;
    s->sr = 0x00000002;
    s->dr = 0x0000000C;
    s->csctrl = 0;

    g233_spi_update_cs(s);
    g233_spi_update_irq(s);
}

static const VMStateDescription vmstate_g233_spi = {
    .name = TYPE_G233_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_FIFO8(tx_fifo, G233SPIState),
        VMSTATE_FIFO8(rx_fifo, G233SPIState),
        VMSTATE_UINT32(cr1, G233SPIState),
        VMSTATE_UINT32(cr2, G233SPIState),
        VMSTATE_UINT32(sr, G233SPIState),
        VMSTATE_UINT32(dr, G233SPIState),
        VMSTATE_UINT32(csctrl, G233SPIState),
        VMSTATE_END_OF_LIST()
    }
};

static void g233_spi_class_init(ObjectClass *klass, const void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, g233_spi_reset);
    dc->realize = g233_spi_realize;
    dc->vmsd = &vmstate_g233_spi;
}

static const TypeInfo g233_spi_info = {
    .name           = TYPE_G233_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(G233SPIState),
    .class_init     = g233_spi_class_init,
};

static void g233_spi_register_types(void)
{
    type_register_static(&g233_spi_info);
}

type_init(g233_spi_register_types)
