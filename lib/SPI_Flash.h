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
    FLASH_OP_NONE,
    FLASH_OP_WRITE_ENABLE,
    FLASH_OP_READ_STATUS,
    FLASH_OP_PAGE_PROGRAM,
    FLASH_OP_READ_DATA,
    FLASH_OP_SECTOR_ERASE,
    FLASH_OP_READ_ID
} flash_operation_t;


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
flash_operation_t get_flash_operation_type(void);
void set_flash_operation_complete(bool complete);
void set_flash_operation_type(flash_operation_t type);
bool is_flash_operation_complete(void);

flash_status_t mx25l_init(void);
uint8_t spi_transfer(uint8_t data);
void delay_us(uint32_t us);
void read_flash_id_sequence(void);
flash_status_t flash_read_word(uint32_t address);

#endif