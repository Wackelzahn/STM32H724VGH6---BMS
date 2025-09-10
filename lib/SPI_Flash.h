#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include "registers.h"


// MX25L12833FM2I Commands
#define MX25L_CMD_READ          0x03
#define MX25L_CMD_FAST_READ     0x0B
#define MX25L_CMD_RDSR          0x05
#define MX25L_CMD_WREN          0x06
#define MX25L_CMD_WRDI          0x04
#define MX25L_CMD_PP            0x02
#define MX25L_CMD_SE            0x20
#define MX25L_CMD_BE            0xD8
#define MX25L_CMD_CE            0xC7
#define MX25L_CMD_RDID          0x9F

// Status Register bits
#define MX25L_SR_WIP            0x01
#define MX25L_SR_WEL            0x02

// Flash parameters
#define MX25L_PAGE_SIZE         256
#define MX25L_SECTOR_SIZE       4096
#define MX25L_BLOCK_SIZE        65536
#define MX25L_CHIP_SIZE         16777216

// CS pin control
#define CS_LOW()    (GPIOE->BSRR = (1 << (4 + 16)))  // Reset PE4
#define CS_HIGH()   (GPIOE->BSRR = (1 << 4))         // Set PE4





typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR = 1,
    FLASH_TIMEOUT = 2
} flash_status_t;



// Structure to hold Flash ID response
typedef struct {
    uint8_t trash;
    uint8_t manufacturer_id;
    uint8_t device_id1;
    uint8_t device_id2;
} flash_id_t;





// Function prototypes
// --------------------------------------
// How to use the functions in main:
// 1. Operations That MUST Wait Internally
// Keep internal waiting for operations that need to poll the flash status register:
// flash_write_page() - needs to wait for WIP bit
// flash_erase_sector() - needs to wait for WIP bit
// flash_wait_ready() - by definition
// --------------------------------------
// 2. Operations That Do NOT Wait Internally
// they must be checked externally for completion:
// while (!is_flash_operation_complete()); // after read operation is complete 
// flash_read_word() - read operation, must check completion externally
// read_flash_id_sequence() - must check completion externally
// flash_read_status() - must check completion externally
// flash_write_enable() - must check completion externally
// --------------------------------------

void delay_us(uint32_t us);
bool is_flash_operation_complete(void);
void return_data_word(uint8_t* data_buffer);
flash_status_t mx25l_init(void);
flash_status_t read_flash_id_sequence(void);
flash_status_t flash_wait_ready(void);
flash_status_t flash_erase_sector(uint32_t address);
flash_status_t flash_read_status(uint8_t* status);
flash_status_t flash_write_enable(void);
flash_status_t flash_write_page(uint32_t address, const uint8_t* data, uint16_t length);
flash_status_t flash_read_word(uint32_t address);

void SPI4_IRQHandler(void);

#endif