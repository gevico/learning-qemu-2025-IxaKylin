/*
 * Test SPI CS functionality for G233 platform
 *
 * Copyright (c) 2025 hongquan.li hongquan.prog@gmail.com
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "crt.h"

/* G233 SPI register definitions */
#define G233_SPI_BASE_ADDR 0x10018000

/* SPI Control Register 1 (SPI_CR1) */
#define SPI_CR1_OFFSET      0x00
#define SPI_CR1_CPHA        (1 << 0)   /* Clock Phase */
#define SPI_CR1_CPOL        (1 << 1)   /* Clock Polarity */
#define SPI_CR1_MSTR        (1 << 2)   /* Master Selection */
#define SPI_CR1_BR_SHIFT    3          /* Baud Rate Control shift */
#define SPI_CR1_SPE         (1 << 6)   /* SPI Enable */
#define SPI_CR1_LSBFIRST    (1 << 7)   /* Frame Format */
#define SPI_CR1_SSI         (1 << 8)   /* Internal Slave Select */
#define SPI_CR1_SSM         (1 << 9)   /* Software Slave Management */
#define SPI_CR1_RXONLY      (1 << 10)  /* Receive Only */
#define SPI_CR1_DFF         (1 << 11)  /* Data Frame Format */
#define SPI_CR1_CRCNEXT     (1 << 12)  /* CRC Transfer Next */
#define SPI_CR1_CRCEN       (1 << 13)  /* Hardware CRC Calculation Enable */
#define SPI_CR1_BIDIOE      (1 << 14)  /* Output Enable in Bidirectional Mode */
#define SPI_CR1_BIDIMODE    (1 << 15)  /* Bidirectional Data Mode Enable */

/* SPI Control Register 2 (SPI_CR2) */
#define SPI_CR2_OFFSET      0x04
#define SPI_CR2_TXEIE        (1 << 7)   /* TXE interrupt enable */
#define SPI_CR2_RXNEIE       (1 << 6)   /* RXNE interrupt enable */
#define SPI_CR2_ERRIE        (1 << 5)   /* Error interrupt enable */
#define SPI_CR2_SSOE         (1 << 4)   /* Software slave select output enable */

/* SPI Status Register (SPI_SR) */
#define SPI_SR_OFFSET       0x08
#define SPI_SR_RXNE         (1 << 0)   /* Receive buffer Not Empty */
#define SPI_SR_TXE          (1 << 1)   /* Transmit buffer Empty */
#define SPI_SR_CHSIDE       (1 << 2)   /* Channel side */
#define SPI_SR_UDR          (1 << 3)   /* Underrun flag */
#define SPI_SR_CRCERR       (1 << 4)   /* CRC error flag */
#define SPI_SR_MODF         (1 << 5)   /* Mode fault */
#define SPI_SR_OVR          (1 << 6)   /* Overrun flag */
#define SPI_SR_BSY          (1 << 7)   /* Busy flag */

/* SPI Data Register (SPI_DR) */
#define SPI_DR_OFFSET       0x0C

/* SPI CS Control Register (SPI_CSCTRL) */
#define SPI_CSCTRL_OFFSET   0x10
#define SPI_CSCTRL_CS0_EN   (1 << 0)   /* CS0 Enable */
#define SPI_CSCTRL_CS1_EN   (1 << 1)   /* CS1 Enable */
#define SPI_CSCTRL_CS2_EN   (1 << 2)   /* CS2 Enable */
#define SPI_CSCTRL_CS3_EN   (1 << 3)   /* CS3 Enable */
#define SPI_CSCTRL_CS0_ACT  (1 << 4)   /* CS0 Active */
#define SPI_CSCTRL_CS1_ACT  (1 << 5)   /* CS1 Active */
#define SPI_CSCTRL_CS2_ACT  (1 << 6)   /* CS2 Active */
#define SPI_CSCTRL_CS3_ACT  (1 << 7)   /* CS3 Active */

/* Memory-mapped register access */
#define REG32(addr) (*(volatile uint32_t *)(G233_SPI_BASE_ADDR + (addr)))

/* W25X Flash Commands */
#define W25X_READ_DATA       0x03  /* Read Data */
#define W25X_READ_STATUS     0x05  /* Read Status Register */
#define W25X_WRITE_ENABLE    0x06  /* Write Enable */
#define W25X_PAGE_PROGRAM    0x02  /* Page Program */
#define W25X_SECTOR_ERASE    0x20  /* Sector Erase (4KB) */
#define W25X_READ_ID         0x9F  /* Read ID */
#define W25X_READ_ID_JEDEC   0x9F  /* Read JEDEC ID */

/* Flash chip definitions */
#define W25X16_ID            0xEF3015  /* W25X16 JEDEC ID */
#define W25X32_ID            0xEF3016  /* W25X32 JEDEC ID */
#define W25X16_SIZE          (2 * 1024 * 1024)  /* 2MB */
#define W25X32_SIZE          (4 * 1024 * 1024)  /* 4MB */

/* Flash chip select definitions */
#define FLASH_CS0              0
#define FLASH_CS1              1

/* Test data patterns */
#define TEST_PATTERN_A         0xAA
#define TEST_PATTERN_B         0x55
#define TEST_PATTERN_C         0x33
#define TEST_PATTERN_D         0xCC

static void g233_spi_init(void)
{
    /* Reset SPI */
    REG32(SPI_CR1_OFFSET) = 0;
    
    /* Configure SPI as master, 8-bit data, MSB first */
    uint32_t cr1 = SPI_CR1_MSTR |           /* Master mode */
                   (0x3 << SPI_CR1_BR_SHIFT) | /* Baud rate: fPCLK/16 */
                   SPI_CR1_SSM |            /* Software slave management */
                   SPI_CR1_SSI |            /* Internal slave select */
                   SPI_CR1_SPE;             /* Enable SPI */
    
    REG32(SPI_CR1_OFFSET) = cr1;
    
    printf("G233 SPI initialized for dual flash operation\n");
}

static void g233_spi_cs_assert(int cs)
{
    uint32_t ctrl = 0;
    
    if (cs == FLASH_CS0) {
        ctrl = SPI_CSCTRL_CS0_EN | SPI_CSCTRL_CS0_ACT;
    } else if (cs == FLASH_CS1) {
        ctrl = SPI_CSCTRL_CS1_EN | SPI_CSCTRL_CS1_ACT;
    }
    
    REG32(SPI_CSCTRL_OFFSET) = ctrl;
}

static void g233_spi_cs_deassert(void)
{
    /* Disable all CS lines */
    REG32(SPI_CSCTRL_OFFSET) = 0;
}

static uint8_t g233_spi_transfer(uint8_t data)
{
    uint32_t sr;
    int retry_count = 0;
    
    /* Wait for TX buffer to be empty */
    do {
        sr = REG32(SPI_SR_OFFSET);
        retry_count++;
        if (retry_count > 1000) {
            printf("Warning: TX buffer empty timeout\n");
            break;
        }
    } while (!(sr & SPI_SR_TXE));
    
    /* Send data */
    REG32(SPI_DR_OFFSET) = data;
    
    /* Wait for RX buffer to have data */
    retry_count = 0;
    do {
        sr = REG32(SPI_SR_OFFSET);
        retry_count++;
        if (retry_count > 1000) {
            printf("Warning: RX buffer not empty timeout\n");
            break;
        }
    } while (!(sr & SPI_SR_RXNE));
    
    /* Return received data */
    return REG32(SPI_DR_OFFSET) & 0xFF;
}

static uint8_t flash_read_status(int cs)
{
    uint8_t cmd = W25X_READ_STATUS;
    uint8_t status;
    
    g233_spi_cs_assert(cs);
    for (volatile int i = 0; i < 1000; i++);
    
    g233_spi_transfer(cmd);
    status = g233_spi_transfer(0x00);
    
    for (volatile int i = 0; i < 1000; i++);
    g233_spi_cs_deassert();
    
    return status;
}

static void flash_write_enable(int cs)
{
    uint8_t cmd = W25X_WRITE_ENABLE;
    
    g233_spi_cs_assert(cs);
    for (volatile int i = 0; i < 1000; i++);
    
    g233_spi_transfer(cmd);
    
    for (volatile int i = 0; i < 1000; i++);
    g233_spi_cs_deassert();
}

static void flash_wait_busy(int cs)
{
    uint8_t status;
    int retry_count = 0;
    
    do {
        status = flash_read_status(cs);
        retry_count++;
        if (retry_count > 10000) {
            printf("Warning: Flash %d busy timeout\n", cs);
            break;
        }
        for (volatile int i = 0; i < 1000; i++); /* Small delay */
    } while (status & 0x01); /* Check BUSY bit */
}

static uint32_t flash_read_id(int cs)
{
    uint8_t cmd = W25X_READ_ID;
    uint32_t id = 0;
    
    g233_spi_cs_assert(cs);
    for (volatile int i = 0; i < 1000; i++);
    
    g233_spi_transfer(cmd);
    id = (g233_spi_transfer(0x00) << 16) |
         (g233_spi_transfer(0x00) << 8) |
         g233_spi_transfer(0x00);
    
    for (volatile int i = 0; i < 1000; i++);
    g233_spi_cs_deassert();
    
    return id;
}

static void flash_sector_erase(int cs, uint32_t addr)
{
    uint8_t cmd = W25X_SECTOR_ERASE;
    uint8_t addr_bytes[3];
    
    printf("Erasing sector at address 0x%06X on Flash %d...\n", addr, cs);
    
    /* Convert address to bytes */
    addr_bytes[0] = (addr >> 16) & 0xFF;
    addr_bytes[1] = (addr >> 8) & 0xFF;
    addr_bytes[2] = addr & 0xFF;
    
    /* Enable write */
    flash_write_enable(cs);
    
    g233_spi_cs_assert(cs);
    for (volatile int i = 0; i < 1000; i++);
    
    /* Send erase command and address */
    g233_spi_transfer(cmd);
    g233_spi_transfer(addr_bytes[0]);
    g233_spi_transfer(addr_bytes[1]);
    g233_spi_transfer(addr_bytes[2]);
    
    for (volatile int i = 0; i < 1000; i++);
    g233_spi_cs_deassert();
    
    /* Wait for erase to complete */
    flash_wait_busy(cs);
    printf("Sector erase completed on Flash %d\n", cs);
}

static void flash_page_program(int cs, uint32_t addr, const uint8_t *data, int len)
{
    uint8_t cmd = W25X_PAGE_PROGRAM;
    uint8_t addr_bytes[3];
    
    printf("Programming page at address 0x%06X on Flash %d, length %d bytes...\n", addr, cs, len);
    
    /* Convert address to bytes */
    addr_bytes[0] = (addr >> 16) & 0xFF;
    addr_bytes[1] = (addr >> 8) & 0xFF;
    addr_bytes[2] = addr & 0xFF;
    
    /* Enable write */
    flash_write_enable(cs);
    
    g233_spi_cs_assert(cs);
    for (volatile int i = 0; i < 1000; i++);
    
    /* Send program command and address */
    g233_spi_transfer(cmd);
    g233_spi_transfer(addr_bytes[0]);
    g233_spi_transfer(addr_bytes[1]);
    g233_spi_transfer(addr_bytes[2]);
    
    /* Send data */
    for (int i = 0; i < len; i++) {
        g233_spi_transfer(data[i]);
    }
    
    for (volatile int i = 0; i < 1000; i++);
    g233_spi_cs_deassert();
    
    /* Wait for program to complete */
    flash_wait_busy(cs);
    printf("Page program completed on Flash %d\n", cs);
}

static void flash_read_data(int cs, uint32_t addr, uint8_t *data, int len)
{
    uint8_t cmd = W25X_READ_DATA;
    uint8_t addr_bytes[3];
    
    /* Convert address to bytes */
    addr_bytes[0] = (addr >> 16) & 0xFF;
    addr_bytes[1] = (addr >> 8) & 0xFF;
    addr_bytes[2] = addr & 0xFF;
    
    g233_spi_cs_assert(cs);
    for (volatile int i = 0; i < 1000; i++);  /* Delay after CS assert */
    
    /* Send read command and address */
    g233_spi_transfer(cmd);
    g233_spi_transfer(addr_bytes[0]);
    g233_spi_transfer(addr_bytes[1]);
    g233_spi_transfer(addr_bytes[2]);
    
    /* Read data */
    for (int i = 0; i < len; i++) {
        data[i] = g233_spi_transfer(0x00);
    }
    
    for (volatile int i = 0; i < 1000; i++);  /* Delay before CS deassert */
    g233_spi_cs_deassert();
}

static void test_flash_identification(void)
{
    uint32_t expected_id[] = {W25X16_ID, W25X32_ID};
    const char* flash_names[] = {"W25X16 (2MB)", "W25X32 (4MB)"};
    
    printf("\n=== Flash Identification Test ===\n");
    
    for (int cs = 0; cs < 2; cs++) {
        uint32_t id = flash_read_id(cs);
        printf("Flash %d ID: 0x%06X\n", cs, id);
        
        if (id == expected_id[cs]) {
            printf("✓ Flash %d: %s detected correctly\n", cs, flash_names[cs]);
        } else {
            printf("✗ Flash %d: Unexpected ID (expected 0x%06X for %s)\n", 
                   cs, expected_id[cs], flash_names[cs]);
        }
    }
}

static void test_individual_flash_operations(void)
{
    printf("\n=== Individual Flash Operations Test ===\n");
    
    uint8_t test_data[256];
    uint8_t read_data[256];
    int errors;
    
    /* Test Flash 0 */
    printf("\n--- Testing Flash 0 ---\n");
    
    /* Generate test data for Flash 0 */
    for (int i = 0; i < 256; i++) {
        test_data[i] = 'A' + (i % 26);  /* Pattern A-Z */
    }
    
    /* Erase, program, and read Flash 0 */
    flash_sector_erase(FLASH_CS0, 0x000000);
    flash_page_program(FLASH_CS0, 0x000000, test_data, 256);
    flash_read_data(FLASH_CS0, 0x000000, read_data, 256);
    
    /* Verify Flash 0 data */
    errors = 0;
    for (int i = 0; i < 256; i++) {
        if (test_data[i] != read_data[i]) {
            errors++;
        }
    }
    
    if (errors == 0) {
        printf("✓ Flash 0: All 256 bytes match perfectly!\n");
    } else {
        printf("✗ Flash 0: %d bytes don't match\n", errors);
    }
    
    /* Test Flash 1 */
    printf("\n--- Testing Flash 1 ---\n");
    
    /* Generate different test data for Flash 1 */
    for (int i = 0; i < 256; i++) {
        test_data[i] = 'a' + (i % 26);  /* Pattern a-z */
    }
    
    /* Erase, program, and read Flash 1 */
    flash_sector_erase(FLASH_CS1, 0x000000);
    flash_page_program(FLASH_CS1, 0x000000, test_data, 256);
    flash_read_data(FLASH_CS1, 0x000000, read_data, 256);
    
    /* Verify Flash 1 data */
    errors = 0;
    for (int i = 0; i < 256; i++) {
        if (test_data[i] != read_data[i]) {
            errors++;
        }
    }
    
    if (errors == 0) {
        printf("✓ Flash 1: All 256 bytes match perfectly!\n");
    } else {
        printf("✗ Flash 1: %d bytes don't match\n", errors);
    }
}

static void test_cross_flash_operations(void)
{
    printf("\n=== Cross Flash Operations Test ===\n");
    
    uint8_t test_data[128];
    uint8_t read_data[128];
    int errors;
    
    /* Write different patterns to different addresses on both flashes */
    printf("\n--- Writing different patterns to both flashes ---\n");
    
    /* Flash 0: Write pattern A at address 0x1000 */
    for (int i = 0; i < 128; i++) {
        test_data[i] = TEST_PATTERN_A;
    }
    flash_sector_erase(FLASH_CS0, 0x001000);
    flash_page_program(FLASH_CS0, 0x001000, test_data, 128);
    
    /* Flash 1: Write pattern B at address 0x1000 */
    for (int i = 0; i < 128; i++) {
        test_data[i] = TEST_PATTERN_B;
    }
    flash_sector_erase(FLASH_CS1, 0x001000);
    flash_page_program(FLASH_CS1, 0x001000, test_data, 128);
    
    /* Flash 0: Write pattern C at address 0x2000 */
    for (int i = 0; i < 128; i++) {
        test_data[i] = TEST_PATTERN_C;
    }
    flash_sector_erase(FLASH_CS0, 0x002000);
    flash_page_program(FLASH_CS0, 0x002000, test_data, 128);
    
    /* Flash 1: Write pattern D at address 0x2000 */
    for (int i = 0; i < 128; i++) {
        test_data[i] = TEST_PATTERN_D;
    }
    flash_sector_erase(FLASH_CS1, 0x002000);
    flash_page_program(FLASH_CS1, 0x002000, test_data, 128);
    
    printf("\n--- Verifying cross-flash data integrity ---\n");
    
    /* Verify Flash 0 data */
    flash_read_data(FLASH_CS0, 0x001000, read_data, 128);
    errors = 0;
    for (int i = 0; i < 128; i++) {
        if (read_data[i] != TEST_PATTERN_A) {
            errors++;
        }
    }
    printf("Flash 0 @ 0x1000: %s (expected pattern A)\n", 
           errors == 0 ? "✓ PASS" : "✗ FAIL");
    
    flash_read_data(FLASH_CS0, 0x002000, read_data, 128);
    errors = 0;
    for (int i = 0; i < 128; i++) {
        if (read_data[i] != TEST_PATTERN_C) {
            errors++;
        }
    }
    printf("Flash 0 @ 0x2000: %s (expected pattern C)\n", 
           errors == 0 ? "✓ PASS" : "✗ FAIL");
    
    /* Verify Flash 1 data */
    flash_read_data(FLASH_CS1, 0x001000, read_data, 128);
    errors = 0;
    for (int i = 0; i < 128; i++) {
        if (read_data[i] != TEST_PATTERN_B) {
            errors++;
        }
    }
    printf("Flash 1 @ 0x1000: %s (expected pattern B)\n", 
           errors == 0 ? "✓ PASS" : "✗ FAIL");
    
    flash_read_data(FLASH_CS1, 0x002000, read_data, 128);
    errors = 0;
    for (int i = 0; i < 128; i++) {
        if (read_data[i] != TEST_PATTERN_D) {
            errors++;
        }
    }
    printf("Flash 1 @ 0x2000: %s (expected pattern D)\n", 
           errors == 0 ? "✓ PASS" : "✗ FAIL");
}

static void test_alternating_operations(void)
{
    printf("\n=== Alternating Operations Test ===\n");
    
    uint8_t test_data[64];
    uint8_t read_data[64];
    int errors;
    
    printf("Testing alternating read/write operations between flashes...\n");
    
    /* Initialize both flashes with different data */
    for (int i = 0; i < 64; i++) {
        test_data[i] = 0x11;  /* Pattern for Flash 0 */
    }
    flash_sector_erase(FLASH_CS0, 0x003000);
    flash_page_program(FLASH_CS0, 0x003000, test_data, 64);
    
    for (int i = 0; i < 64; i++) {
        test_data[i] = 0x22;  /* Pattern for Flash 1 */
    }
    flash_sector_erase(FLASH_CS1, 0x003000);
    flash_page_program(FLASH_CS1, 0x003000, test_data, 64);
    
    /* Alternating read operations */
    printf("Performing alternating read operations...\n");
    
    for (int round = 0; round < 5; round++) {
        /* Read from Flash 0 */
        flash_read_data(FLASH_CS0, 0x003000, read_data, 64);
        errors = 0;
        for (int i = 0; i < 64; i++) {
            if (read_data[i] != 0x11) errors++;
        }
        printf("Round %d - Flash 0: %s\n", round + 1, 
               errors == 0 ? "✓ PASS" : "✗ FAIL");
        
        /* Read from Flash 1 */
        flash_read_data(FLASH_CS1, 0x003000, read_data, 64);
        errors = 0;
        for (int i = 0; i < 64; i++) {
            if (read_data[i] != 0x22) errors++;
        }
        printf("Round %d - Flash 1: %s\n", round + 1, 
               errors == 0 ? "✓ PASS" : "✗ FAIL");
    }
}

static void test_flash_capacity(void)
{
    printf("\n=== Flash Capacity Test ===\n");
    
    uint32_t flash_sizes[] = {W25X16_SIZE, W25X32_SIZE};
    const char* flash_names[] = {"W25X16", "W25X32"};
    uint8_t test_data[256];
    uint8_t read_data[256];
    int errors;
    
    for (int cs = 0; cs < 2; cs++) {
        printf("\n--- Testing %s capacity ---\n", flash_names[cs]);
        
        /* Test at different addresses within the flash capacity */
        uint32_t test_addresses[] = {
            0x000000,  /* Start of flash */
            0x100000,  /* 1MB offset */
            (cs == 0) ? 0x1F0000 : 0x3F0000,  /* Near end of flash */
        };
        
        int num_tests = (cs == 0) ? 2 : 3;  /* W25X16 has 2MB, W25X32 has 4MB */
        
        for (int test = 0; test < num_tests; test++) {
            uint32_t addr = test_addresses[test];
            
            /* Generate test data */
            for (int i = 0; i < 256; i++) {
                test_data[i] = (addr >> 16) + (i % 256);
            }
            
            printf("Testing at address 0x%06X...\n", addr);
            
            /* Erase, program, and read */
            flash_sector_erase(cs, addr);
            flash_page_program(cs, addr, test_data, 256);
            flash_read_data(cs, addr, read_data, 256);
            
            /* Verify data */
            errors = 0;
            for (int i = 0; i < 256; i++) {
                if (test_data[i] != read_data[i]) {
                    errors++;
                }
            }
            
            if (errors == 0) {
                printf("✓ Address 0x%06X: PASS\n", addr);
            } else {
                printf("✗ Address 0x%06X: FAIL (%d errors)\n", addr, errors);
            }
        }
    }
}

static void test_concurrent_status_check(void)
{
    printf("\n=== Concurrent Status Check Test ===\n");
    
    printf("Checking status of both flashes simultaneously...\n");
    
    for (int i = 0; i < 10; i++) {
        uint8_t status0 = flash_read_status(FLASH_CS0);
        uint8_t status1 = flash_read_status(FLASH_CS1);
        
        printf("Status check %d: Flash0=0x%02X, Flash1=0x%02X\n", 
               i + 1, status0, status1);
        
        /* Both flashes should be ready (not busy) */
        if ((status0 & 0x01) == 0 && (status1 & 0x01) == 0) {
            printf("✓ Both flashes ready\n");
        } else {
            printf("✗ One or both flashes busy\n");
        }
    }
}

int main(void)
{
    printf("G233 Dual SPI Flash Test\n");
    printf("========================\n");
    
    g233_spi_init();
    
    /* Run all test cases */
    test_flash_identification();
    test_individual_flash_operations();
    test_cross_flash_operations();
    test_alternating_operations();
    test_flash_capacity();
    test_concurrent_status_check();
    
    printf("\n=== Test Summary ===\n");
    printf("Dual flash test completed!\n");
    printf("Flash0 (W25X16) and Flash1 (W25X32 have been tested for:\n");
    printf("- Individual read/write operations\n");
    printf("- Cross-flash data integrity\n");
    printf("- Alternating operations\n");
    printf("- Capacity verification (2MB vs 4MB)\n");
    printf("- Concurrent status monitoring\n");
    
    return 0;
}
