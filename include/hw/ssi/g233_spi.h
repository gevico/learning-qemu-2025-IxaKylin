/*
 * QEMU model of the SiFive SPI Controller
 *
 * Copyright (c) 2021 Wind River Systems, Inc.
 *
 * Author:
 *   Bin Meng <bin.meng@windriver.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_G233_SPI_H
#define HW_G233_SPI_H

#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"
#include "qemu/fifo8.h"


#define G233_CS_NUM           4

#define TYPE_G233_SPI "g233-spi"
OBJECT_DECLARE_SIMPLE_TYPE(G233SPIState, G233_SPI)

#define FIFO_SIZE               64
#define FIFO_SIZE_3_4           48

#define G233_SPI_CR1        0x00
#define G233_SPI_CR2        0x04
#define G233_SPI_SR         0x08
#define G233_SPI_DR         0x0C
#define G233_SPI_CSCTRL     0x10

#define G233_SPI_CR1_MSTR         BIT(2)
#define G233_SPI_CR1_SPE          BIT(6)

#define G233_SPI_CR2_SSOE         BIT(4)
#define G233_SPI_CR2_ERRIE        BIT(5)
#define G233_SPI_CR2_RXNEIE       BIT(6)
#define G233_SPI_CR2_TXEIE        BIT(7)

#define G233_SPI_SR_RXNE          BIT(0)
#define G233_SPI_SR_TXE           BIT(1)
#define G233_SPI_SR_UNDERRUN      BIT(2)
#define G233_SPI_SR_OVERRUN       BIT(3)
#define G233_SPI_SR_BUSY          BIT(7)

#define G233_SPI_CS_EN(x)        ((1 << x) & 0xf)
#define G233_SPI_CS_ACT(x)       (((1 << x) & 0xf) << 4)
#define G233_SPI_CS_GET_ACT(x, n)   ((x >> 4) & (1 << n))


struct G233SPIState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    SSIBus *bus;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t num_cs;
    qemu_irq cs_lines[4];

    uint32_t cr1;
    uint32_t cr2;
    uint32_t sr;
    uint32_t dr;
    uint32_t csctrl;

    Fifo8 rx_fifo;
    Fifo8 tx_fifo;
};

#endif