/*
 * Test SPI OVERRUN interrupt detection
 *
 * Copyright (c) 2025 hongquan.li hongquan.prog@gmail.com
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "crt.h"

#define G233_SPI0_BASE 0x10018000

/* Memory-mapped register access */
#define REG32(addr) (*(volatile uint32_t *)((uintptr_t)(G233_SPI0_BASE + (addr))))

/* G233 SPI register offsets */
#define SPI_CR1     0x00
#define SPI_CR2     0x04
#define SPI_SR      0x08
#define SPI_DR      0x0C
#define SPI_CSCTRL  0x10

/* SPI Control Register 1 (CR1) bits */
#define SPI_CR1_SPE     (1 << 6)   /* SPI Enable */
#define SPI_CR1_MSTR    (1 << 2)   /* Master mode */
#define SPI_CR1_BR_0    (1 << 3)   /* Baud rate control bit 0 */
#define SPI_CR1_BR_1    (1 << 4)   /* Baud rate control bit 1 */
#define SPI_CR1_BR_2    (1 << 5)   /* Baud rate control bit 2 */

/* SPI Control Register 2 (CR2) bits */
#define SPI_CR2_TXEIE   (1 << 7)   /* TX buffer empty interrupt enable */
#define SPI_CR2_RXNEIE  (1 << 6)   /* RX buffer not empty interrupt enable */
#define SPI_CR2_ERRIE   (1 << 5)   /* Error interrupt enable */

/* SPI Status Register (SR) bits */
#define SPI_SR_TXE      (1 << 1)   /* Transmit buffer empty */
#define SPI_SR_RXNE     (1 << 0)   /* Receive buffer not empty */
#define SPI_SR_OVERRUN  (1 << 3)   /* Overrun error flag */
#define SPI_SR_UNDERRUN (1 << 2)   /* Underrun error flag */
#define SPI_SR_BSY      (1 << 7)   /* Busy flag */

/* CS Control Register bits */
#define SPI_CS_ENABLE   (1 << 0)   /* Enable CS0 */
#define SPI_CS_ACTIVE   (1 << 4)   /* Activate CS0 */

/* Global variables for interrupt handling */
static volatile int overrun_detected = 0;

static void spi_write_reg(uint32_t addr, uint32_t value)
{
    REG32(addr) = value;
}

static uint32_t spi_read_reg(uint32_t addr)
{
    return REG32(addr);
}

static void spi_wait_txe(void)
{
    uint32_t sr;
    do {
        sr = spi_read_reg(SPI_SR);
    } while (!(sr & SPI_SR_TXE));
}

static void spi_wait_rxne(void)
{
    uint32_t sr;
    do {
        sr = spi_read_reg(SPI_SR);
    } while (!(sr & SPI_SR_RXNE));
}

static void spi_wait_not_busy(void)
{
    uint32_t sr;
    do {
        sr = spi_read_reg(SPI_SR);
    } while (sr & SPI_SR_BSY);
}

static uint8_t spi_transfer_byte(uint8_t data)
{
    /* Wait for TX buffer to be empty */
    spi_wait_txe();
    
    /* Write data to DR register */
    spi_write_reg(SPI_DR, data);
    
    /* Wait for RX buffer to have data */
    spi_wait_rxne();
    
    /* Read received data */
    return spi_read_reg(SPI_DR) & 0xFF;
}

static void spi_cs_assert(void)
{
    /* Enable and activate CS0 */
    spi_write_reg(SPI_CSCTRL, SPI_CS_ENABLE | SPI_CS_ACTIVE);
}

static void spi_cs_deassert(void)
{
    /* Deactivate CS0 but keep it enabled */
    spi_write_reg(SPI_CSCTRL, SPI_CS_ENABLE);
}

/* SPI Interrupt Handler */
void spi0_interrupt_handler(void)
{
    uint32_t sr = spi_read_reg(SPI_SR);
    uint32_t cr2 = spi_read_reg(SPI_CR2);
    
    printf("SPI Interrupt: SR=0x%02X, CR2=0x%02X\n", sr, cr2);
    
    /* Check for error conditions first */
    if ((cr2 & SPI_CR2_ERRIE) && (sr & (SPI_SR_UNDERRUN | SPI_SR_OVERRUN))) {
        printf("  Error interrupt: UDR=%d, OVR=%d\n", 
               !!(sr & SPI_SR_UNDERRUN), !!(sr & SPI_SR_OVERRUN));
        
        if (sr & SPI_SR_OVERRUN) {
            spi_write_reg(SPI_SR, SPI_SR_OVERRUN); 
            overrun_detected = 1;
            printf("  OVERRUN detected in interrupt!\n");
        }
    }
}

static void spi_init_interrupt(void)
{
    /* Reset SPI */
    spi_write_reg(SPI_CR1, 0x00000000);
    spi_write_reg(SPI_CR2, 0x00000000);
    spi_write_reg(SPI_CSCTRL, 0x00000000);
    
    /* Configure SPI as master, enable SPI, set baud rate */
    spi_write_reg(SPI_CR1, SPI_CR1_MSTR | SPI_CR1_SPE | 
                  SPI_CR1_BR_2); /* Slow clock for testing */
    
    /* Enable interrupts: RXNE, TXE, and Error interrupts */
    spi_write_reg(SPI_CR2,  SPI_CR2_ERRIE);
    
    /* Wait for initialization */
    spi_wait_not_busy();
    
    /* Reset interrupt flags */
    overrun_detected = 0;
    
    /* Debug: Print SPI configuration */
    printf("SPI CR1: 0x%08X\n", spi_read_reg(SPI_CR1));
    printf("SPI CR2: 0x%08X\n", spi_read_reg(SPI_CR2));
    printf("SPI SR: 0x%08X\n", spi_read_reg(SPI_SR));
}

static void test_interrupt_overrun_detection(void)
{
    uint32_t sr;
    int timeout_count = 0;
    const int max_timeout = 10000;
    
    printf("\nTesting OVERRUN detection with interrupts...\n");
    
    /* Initialize SPI with interrupts enabled */
    spi_init_interrupt();
    spi_cs_assert();
    
    /* Reset interrupt flags */
    overrun_detected = 0;
    
    /* Step 1: Send first byte and DON'T read it (leave RXNE set) */
    printf("\n1. Sending first byte without reading (RXNE should be set)...\n");
    spi_wait_txe();
    spi_write_reg(SPI_DR, 0xAA);
    spi_wait_rxne();  /* Wait for RXNE to be set */
    
    sr = spi_read_reg(SPI_SR);
    crt_assert(sr & SPI_SR_RXNE);  /* RXNE should be set */
    printf("   ✓ RXNE flag set (data ready to be read)\n");
    
    /* Step 2: Send second byte while RXNE is still set - this should trigger OVERRUN */
    printf("\n2. Sending second byte while RXNE is set (should trigger OVERRUN)...\n");
    
    /* Reset interrupt flags */
    overrun_detected = 0;

    /* Send another byte while RXNE is still set - this should trigger OVERRUN */
    printf("   Sending byte while RXNE is set (should trigger OVERRUN)...\n");
    spi_wait_txe();
    // spi_write_reg(SPI_DR, 0x55);
    REG32(SPI_DR) = 0x55;

    if (overrun_detected) {
        printf("   ✓ OVERRUN detected in interrupt!\n");
    } else {
        printf("   ! OVERRUN not detected in interrupt\n");
        crt_assert(0);
    }
    
    spi_cs_deassert();
}

static void test_polling_overrun_detection(void)
{
    uint32_t sr;
    int timeout_count = 0;
    const int max_timeout = 10000;
    
    printf("\nTesting OVERRUN detection with polling...\n");
    
    /* Initialize SPI with interrupts DISABLED */
    spi_write_reg(SPI_CR2, 0x00000000);
    spi_cs_assert();
    
    /* Step 1: Send first byte and DON'T read it (leave RXNE set) */
    printf("\n1. Sending first byte without reading (RXNE should be set)...\n");
    spi_wait_txe();
    spi_write_reg(SPI_DR, 0xAA);
    spi_wait_rxne();  /* Wait for RXNE to be set */
    
    sr = spi_read_reg(SPI_SR);
    crt_assert(sr & SPI_SR_RXNE);  /* RXNE should be set */
    printf("   ✓ RXNE flag set (data ready to be read)\n");
    
    /* Step 2: Send second byte while RXNE is still set - this should trigger OVERRUN */
    printf("\n2. Sending second byte while RXNE is set (should trigger OVERRUN)...\n");
    
    /* Send another byte while RXNE is still set - this should trigger OVERRUN */
    printf("   Sending byte while RXNE is set (should trigger OVERRUN)...\n");
    spi_wait_txe();
    REG32(SPI_DR) = 0x55;

    sr = spi_read_reg(SPI_SR);
    
    if (sr & SPI_SR_OVERRUN) {
        printf("   ✓ OVERRUN flag detected via polling!\n");
        
        /* Verify that RXNE is still set (data is still there) */
        if (sr & SPI_SR_RXNE) {
            printf("   ✓ RXNE still set (data preserved)\n");
        } else {
            printf("   ! RXNE cleared unexpectedly\n");
            crt_assert(0);
        }
        
        /* Clear OVERRUN flag by write 1 to SR */
        spi_write_reg(SPI_SR, SPI_SR_OVERRUN);
        
        /* Check status after clearing */
        sr = spi_read_reg(SPI_SR);
        printf("   Status after clearing: 0x%02X\n", sr);
        
        if (!(sr & SPI_SR_OVERRUN)) {
            printf("   ✓ OVERRUN flag cleared successfully\n");
        } else {
            printf("   ! OVERRUN flag not cleared\n");
            crt_assert(0);
        }
        
    } else {
        printf("   ! OVERRUN flag not detected via polling\n");
        crt_assert(0);
    }
    
    spi_cs_deassert();
}

int main(void)
{
    printf("STM32F2XX SPI OVERRUN Test\n");
    printf("============================\n");
    
    /* Interrupt-based OVERRUN detection */
    test_interrupt_overrun_detection();
    
    /* Polling-based OVERRUN detection */
    test_polling_overrun_detection();
    
    printf("\nAll OVERRUN tests passed!\n");
    return 0;
}
