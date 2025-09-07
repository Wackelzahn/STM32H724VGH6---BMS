#ifndef REGISTERS_H
#define REGISTERS_H

// Registers for STM32H742

#include <inttypes.h>


// Register PWR (Power Control)  -->checked!
struct pwr {
    volatile uint32_t CR1;      // 0x000
    volatile uint32_t CSR1;     // 0x004  
    volatile uint32_t CR2;      // 0x008
    volatile uint32_t CR3;      // 0x00C
    volatile uint32_t CPUCR;    // 0x010
    volatile uint32_t Reserved0; // 0x014
    volatile uint32_t D3CR;     // 0x018
    volatile uint32_t Reserved1; // 0x01C
    volatile uint32_t WKUPCR;   // 0x020
    volatile uint32_t WKUPFR;   // 0x024
    volatile uint32_t WKUPEPR;  // 0x028
    volatile uint32_t Reserved2; // 0x02C (if needed for alignment)
};
#define PWR ((struct pwr *) 0x58024800) // --> checked!

// Register FLASH for STM32H742
struct flash {
    volatile uint32_t ACR;          // 0x000 - Access control register
    volatile uint32_t KEYR1;        // 0x004 - Key register for bank 1
    volatile uint32_t OPTKEYR;      // 0x008 - Option key register
    volatile uint32_t CR1;          // 0x00C - Control register for bank 1
    volatile uint32_t SR1;          // 0x010 - Status register for bank 1
    volatile uint32_t CCR1;         // 0x014 - Clear control register for bank 1
    volatile uint32_t OPTCR;        // 0x018 - Option control register
    volatile uint32_t OPTSR_CUR;    // 0x01C - Option status current register
    volatile uint32_t OPTSR_PRG;    // 0x020 - Option status program register
    volatile uint32_t OPTCCR;       // 0x024 - Option clear control register
    volatile uint32_t PRAR_CUR1;    // 0x028 - Protection area current register bank 1
    volatile uint32_t PRAR_PRG1;    // 0x02C - Protection area program register bank 1
    volatile uint32_t SCAR_CUR1;    // 0x030 - Secure area current register bank 1
    volatile uint32_t SCAR_PRG1;    // 0x034 - Secure area program register bank 1
    volatile uint32_t WPSN_CUR1R;   // 0x038 - Write protection current register bank 1
    volatile uint32_t WPSN_PRG1R;   // 0x03C - Write protection program register bank 1
    volatile uint32_t BOOT_CURR;    // 0x040 - Boot current register
    volatile uint32_t BOOT_PRGR;    // 0x044 - Boot program register
    volatile uint32_t RESERVED1[2]; // 0x048-0x04C - Reserved
    volatile uint32_t CRCCR1;       // 0x050 - CRC control register for bank 1
    volatile uint32_t CRCSADD1R;    // 0x054 - CRC start address register for bank 1
    volatile uint32_t CRCEADD1R;    // 0x058 - CRC end address register for bank 1
    volatile uint32_t CRCDATAR;     // 0x05C - CRC data register
    volatile uint32_t ECC_FA1R;     // 0x060 - ECC fail address register for bank 1
    volatile uint32_t RESERVED2[39];// 0x064-0x0FC - Reserved
    
    // Bank 2 registers start at offset 0x100
    volatile uint32_t ACR2;         // 0x100 - Access control register (duplicate)
    volatile uint32_t KEYR2;        // 0x104 - Key register for bank 2
    volatile uint32_t OPTKEYR2;     // 0x108 - Option key register (duplicate)
    volatile uint32_t CR2;          // 0x10C - Control register for bank 2
    volatile uint32_t SR2;          // 0x110 - Status register for bank 2
    volatile uint32_t CCR2;         // 0x114 - Clear control register for bank 2
    volatile uint32_t OPTCR2;       // 0x118 - Option control register (duplicate)
    volatile uint32_t OPTSR_CUR2;   // 0x11C - Option status current register (duplicate)
    volatile uint32_t OPTSR_PRG2;   // 0x120 - Option status program register (duplicate)
    volatile uint32_t OPTCCR2;      // 0x124 - Option clear control register (duplicate)
    volatile uint32_t PRAR_CUR2;    // 0x128 - Protection area current register bank 2
    volatile uint32_t PRAR_PRG2;    // 0x12C - Protection area program register bank 2
    volatile uint32_t SCAR_CUR2;    // 0x130 - Secure area current register bank 2
    volatile uint32_t SCAR_PRG2;    // 0x134 - Secure area program register bank 2
    volatile uint32_t WPSN_CUR2R;   // 0x138 - Write protection current register bank 2
    volatile uint32_t WPSN_PRG2R;   // 0x13C - Write protection program register bank 2
    volatile uint32_t BOOT_CURR2;   // 0x140 - Boot current register (duplicate)
    volatile uint32_t BOOT_PRGR2;   // 0x144 - Boot program register (duplicate)
    volatile uint32_t RESERVED3[2]; // 0x148-0x14C - Reserved
    volatile uint32_t CRCCR2;       // 0x150 - CRC control register for bank 2
    volatile uint32_t CRCSADD2R;    // 0x154 - CRC start address register for bank 2
    volatile uint32_t CRCEADD2R;    // 0x158 - CRC end address register for bank 2
    volatile uint32_t CRCDATAR2;    // 0x15C - CRC data register (duplicate)
    volatile uint32_t ECC_FA2R;     // 0x160 - ECC fail address register for bank 2
};

// Flash memory base address for STM32H742 --> checked!
#define FLASH ((struct flash *) 0x52002000)




// Register RCC (Reset and Clock Control) for STM32H7 Rev V  --> checked!
struct rcc {
    volatile uint32_t CR;           // 0x000
    volatile uint32_t HSICFGR;      // 0x004 (Rev V devices)
    volatile uint32_t CRRCR;        // 0x008
    volatile uint32_t CSICFGR;      // 0x00C (Rev V devices)
    volatile uint32_t CFGR;         // 0x010
    volatile uint32_t RESERVED1;    // 0x014
    volatile uint32_t D1CFGR;       // 0x018
    volatile uint32_t D2CFGR;       // 0x01C
    volatile uint32_t D3CFGR;       // 0x020
    volatile uint32_t RESERVED2;    // 0x024
    volatile uint32_t PLLCKSELR;    // 0x028
    volatile uint32_t PLLCFGR;      // 0x02C
    volatile uint32_t PLL1DIVR;     // 0x030
    volatile uint32_t PLL1FRACR;    // 0x034
    volatile uint32_t PLL2DIVR;     // 0x038
    volatile uint32_t PLL2FRACR;    // 0x03C
    volatile uint32_t PLL3DIVR;     // 0x040
    volatile uint32_t PLL3FRACR;    // 0x044
    volatile uint32_t RESERVED3;    // 0x048
    volatile uint32_t D1CCIPR;      // 0x04C
    volatile uint32_t D2CCIP1R;     // 0x050
    volatile uint32_t D2CCIP2R;     // 0x054
    volatile uint32_t D3CCIPR;      // 0x058
    volatile uint32_t RESERVED4;    // 0x05C
    volatile uint32_t CIER;         // 0x060
    volatile uint32_t CIFR;         // 0x064
    volatile uint32_t CICR;         // 0x068
    volatile uint32_t RESERVED5;    // 0x06C
    volatile uint32_t BDCR;         // 0x070
    volatile uint32_t CSR;          // 0x074
    volatile uint32_t RESERVED6;    // 0x078
    volatile uint32_t AHB3RSTR;     // 0x07C
    volatile uint32_t AHB1RSTR;     // 0x080
    volatile uint32_t AHB2RSTR;     // 0x084
    volatile uint32_t AHB4RSTR;     // 0x088
    volatile uint32_t APB3RSTR;     // 0x08C
    volatile uint32_t APB1LRSTR;    // 0x090
    volatile uint32_t APB1HRSTR;    // 0x094
    volatile uint32_t APB2RSTR;     // 0x098
    volatile uint32_t APB4RSTR;     // 0x09C
    volatile uint32_t GCR;          // 0x0A0
    volatile uint32_t RESERVED7;    // 0x0A4
    volatile uint32_t D3AMR;        // 0x0A8
    volatile uint32_t RESERVED8[9]; // 0x0AC to 0x0CC
    volatile uint32_t RSR;          // 0x0D0
    volatile uint32_t AHB3ENR;      // 0x0D4
    volatile uint32_t AHB1ENR;      // 0x0D8
    volatile uint32_t AHB2ENR;      // 0x0DC
    volatile uint32_t AHB4ENR;      // 0x0E0
    volatile uint32_t APB3ENR;      // 0x0E4
    volatile uint32_t APB1LENR;     // 0x0E8
    volatile uint32_t APB1HENR;     // 0x0EC
    volatile uint32_t APB2ENR;      // 0x0F0
    volatile uint32_t APB4ENR;      // 0x0F4
    volatile uint32_t RESERVED9;    // 0x0F8
    volatile uint32_t AHB3LPENR;    // 0x0FC
    volatile uint32_t AHB1LPENR;    // 0x100
    volatile uint32_t AHB2LPENR;    // 0x104
    volatile uint32_t AHB4LPENR;    // 0x108
    volatile uint32_t APB3LPENR;    // 0x10C
    volatile uint32_t APB1LLPENR;   // 0x110
    volatile uint32_t APB1HLPENR;   // 0x114
    volatile uint32_t APB2LPENR;    // 0x118
    volatile uint32_t APB4LPENR;    // 0x11C
    volatile uint32_t RESERVED10[5]; // 0x120 to 0x130
    volatile uint32_t C1_AHB3ENR;   // 0x134
    volatile uint32_t C1_AHB1ENR;   // 0x138
    volatile uint32_t C1_AHB2ENR;   // 0x13C
    volatile uint32_t C1_AHB4ENR;   // 0x140
    volatile uint32_t C1_APB3ENR;   // 0x144
    volatile uint32_t C1_APB1LENR;  // 0x148
    volatile uint32_t C1_APB1HENR;  // 0x14C
    volatile uint32_t C1_APB2ENR;   // 0x150
    volatile uint32_t C1_APB4ENR;   // 0x154
    volatile uint32_t RESERVED11;   // 0x158
    volatile uint32_t C1_AHB3LPENR; // 0x15C
    volatile uint32_t C1_AHB1LPENR; // 0x160
    volatile uint32_t C1_AHB2LPENR; // 0x164
    volatile uint32_t C1_AHB4LPENR; // 0x168
    volatile uint32_t C1_APB3LPENR; // 0x16C
    volatile uint32_t C1_APB1LLPENR;// 0x170
    volatile uint32_t C1_APB1HLPENR;// 0x174
    volatile uint32_t C1_APB2LPENR; // 0x178
    volatile uint32_t C1_APB4LPENR; // 0x17C
    volatile uint32_t RESERVED12[32]; // 0x180 to 0x1FC
};

#define RCC ((struct rcc *) 0x58024400) // --> checked!



// Register I2C  --> checked!
struct I2C {
  volatile uint32_t CR1, CR2, OAR1, OAR2, TIMINGR, TIMEOUTR, ISR, ICR, PECR,
      RXDR, TXDR;
};
#define I2C1 ((struct I2C *) 0x40005400) // checked!
#define I2C2 ((struct I2C *) 0x40005800) // checked!



// Register SYSTICK  --> checked!
struct systick {
  volatile uint32_t CSR, RVR, CVR, CALIB;
};
#define SYST ((struct systick *) 0xE000E010) // --> checked (inside M7 core reference)



// Register NVIC (Nested Vectored Interrupt Controller) for Cortex-M7 --> checked!
struct nvic {
    volatile uint32_t ISER[8];      // 0xE000E100-0xE000E11C: Interrupt Set-Enable Registers
    uint32_t RESERVED0[24];         // 0xE000E120-0xE000E17C: Reserved
    volatile uint32_t ICER[8];      // 0xE000E180-0xE000E19C: Interrupt Clear-Enable Registers  
    uint32_t RESERVED1[24];         // 0xE000E1A0-0xE000E1FC: Reserved
    volatile uint32_t ISPR[8];      // 0xE000E200-0xE000E21C: Interrupt Set-Pending Registers
    uint32_t RESERVED2[24];         // 0xE000E220-0xE000E27C: Reserved
    volatile uint32_t ICPR[8];      // 0xE000E280-0xE000E29C: Interrupt Clear-Pending Registers
    uint32_t RESERVED3[24];         // 0xE000E2A0-0xE000E2FC: Reserved
    volatile uint32_t IABR[8];      // 0xE000E300-0xE000E31C: Interrupt Active Bit Registers
    uint32_t RESERVED4[56];         // 0xE000E320-0xE000E3FC: Reserved
    volatile uint32_t IPR[60];      // 0xE000E400-0xE000E4EF: Interrupt Priority Registers
    uint32_t RESERVED5[644];        // 0xE000E4F0-0xE000EEFC: Reserved
    volatile uint32_t STIR;         // 0xE000EF00: Software Trigger Interrupt Register
};

#define NVIC ((struct nvic *) 0xE000E100) //--> checked!





// Register USART --> checked!
struct usart {
  volatile uint32_t CR1, CR2, CR3, BRR, GTPR, RTOR, RQR, ISR, ICR, RDR, TDR,
      PRESC;
};
#define USART1 ((struct usart *) 0x40011000) // checked!
#define USART2 ((struct usart *) 0x40004400) // checked!
#define USART3 ((struct usart *) 0x40004800) // checked!





// Register FDCAN (Controller Area Network with Flexible Data Rate) for STM32H7  --> checked!
struct fdcan {
    volatile uint32_t CREL;         // 0x0000: Core release register
    volatile uint32_t ENDN;         // 0x0004: Endian register
    volatile uint32_t RESERVED0;    // 0x0008: Reserved
    volatile uint32_t DBTP;         // 0x000C: Data bit timing and prescaler register
    volatile uint32_t TEST;         // 0x0010: Test register
    volatile uint32_t RWD;          // 0x0014: RAM watchdog register
    volatile uint32_t CCCR;         // 0x0018: CC control register
    volatile uint32_t NBTP;         // 0x001C: Nominal bit timing and prescaler register
    volatile uint32_t TSCC;         // 0x0020: Timestamp counter configuration register
    volatile uint32_t TSCV;         // 0x0024: Timestamp counter value register
    volatile uint32_t TOCC;         // 0x0028: Timeout counter configuration register
    volatile uint32_t TOCV;         // 0x002C: Timeout counter value register
    volatile uint32_t RESERVED1[4]; // 0x0030-0x003C: Reserved
    volatile uint32_t ECR;          // 0x0040: Error counter register
    volatile uint32_t PSR;          // 0x0044: Protocol status register
    volatile uint32_t TDCR;         // 0x0048: Transmitter delay compensation register
    volatile uint32_t RESERVED2;    // 0x004C: Reserved
    volatile uint32_t IR;           // 0x0050: Interrupt register
    volatile uint32_t IE;           // 0x0054: Interrupt enable register
    volatile uint32_t ILS;          // 0x0058: Interrupt line select register
    volatile uint32_t ILE;          // 0x005C: Interrupt line enable register
    volatile uint32_t RESERVED3[8]; // 0x0060-0x007C: Reserved
    volatile uint32_t GFC;          // 0x0080: Global filter configuration register
    volatile uint32_t SIDFC;        // 0x0084: Standard ID filter configuration register
    volatile uint32_t XIDFC;        // 0x0088: Extended ID filter configuration register
    volatile uint32_t RESERVED4;    // 0x008C: Reserved
    volatile uint32_t XIDAM;        // 0x0090: Extended ID AND mask register
    volatile uint32_t HPMS;         // 0x0094: High priority message status register
    volatile uint32_t NDAT1;        // 0x0098: New data 1 register
    volatile uint32_t NDAT2;        // 0x009C: New data 2 register
    volatile uint32_t RXF0C;        // 0x00A0: Rx FIFO 0 configuration register
    volatile uint32_t RXF0S;        // 0x00A4: Rx FIFO 0 status register
    volatile uint32_t RXF0A;        // 0x00A8: Rx FIFO 0 acknowledge register
    volatile uint32_t RXBC;         // 0x00AC: Rx buffer configuration register
    volatile uint32_t RXF1C;        // 0x00B0: Rx FIFO 1 configuration register
    volatile uint32_t RXF1S;        // 0x00B4: Rx FIFO 1 status register
    volatile uint32_t RXF1A;        // 0x00B8: Rx FIFO 1 acknowledge register
    volatile uint32_t RXESC;        // 0x00BC: Rx buffer/FIFO element size configuration register
    volatile uint32_t TXBC;         // 0x00C0: Tx buffer configuration register
    volatile uint32_t TXFQS;        // 0x00C4: Tx FIFO/queue status register
    volatile uint32_t TXESC;        // 0x00C8: Tx buffer element size configuration register
    volatile uint32_t TXBRP;        // 0x00CC: Tx buffer request pending register
    volatile uint32_t TXBAR;        // 0x00D0: Tx buffer add request register
    volatile uint32_t TXBCR;        // 0x00D4: Tx buffer cancellation request register
    volatile uint32_t TXBTO;        // 0x00D8: Tx buffer transmission occurred register
    volatile uint32_t TXBCF;        // 0x00DC: Tx buffer cancellation finished register
    volatile uint32_t TXBTIE;       // 0x00E0: Tx buffer transmission interrupt enable register
    volatile uint32_t TXBCIE;       // 0x00E4: Tx buffer cancellation finished interrupt enable register
    volatile uint32_t RESERVED5[2]; // 0x00E8-0x00EC: Reserved
    volatile uint32_t TXEFC;        // 0x00F0: Tx event FIFO configuration register
    volatile uint32_t TXEFS;        // 0x00F4: Tx event FIFO status register
    volatile uint32_t TXEFA;        // 0x00F8: Tx event FIFO acknowledge register
};

#define FDCAN1 ((struct fdcan *) 0x4000A000)  // checked!
#define FDCAN2 ((struct fdcan *) 0x4000A400)  // checked!



// FDCAN Message RAM base address --> checked, needs practical verification
#define FDCAN_MESSAGE_RAM_BASE    0x4000AC00  // checked!
#define FDCAN1_MESSAGE_RAM_BASE_offset 0x0000
#define FDCAN2_MESSAGE_RAM_BASE_offset 0x1400  // gives 0x4000C000
#define FDCAN_Std_Filter_RAM_offset 0x0000  // 28 Words reserved (28 Elements)
#define FDCAN_Ext_Filter_RAM_offset 0x0070  // 16 Words reserved (8 Elements)
#define FDCAN_RX_FIFO0_offset 0x00B0        // 72 Words reserved (18 Elements) --> 4Words per element (minimum for 8byte data)
#define FDCAN_RX_FIFO1_offset 0x01D0        // 72 Words reserved (18 Elements) --> 4Words per element (minimum for 8byte data)
#define FDCAN_RX_BUFFER_offset 0x02F0       // 360 Words reserved (20 Elements)
#define FDCAN_TX_EVENT_FIFO_offset 0x0890   // 6 Words reserved (3 Elements)
#define FDCAN_TX_BUFFER_offset 0x08A8       // 72 Words reserved (18 Elements) --> 4Words per element (minimum for 8byte data)
#define FDCAN_TRIGGER_MEMORY_offset 0x09C8  // 16 Words reserved (8 Elements)

#define FDCAN1_StandartMessageIDFilter ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_Std_Filter_RAM_offset))
#define FDCAN1_ExtendedMessageIDFilter ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_Ext_Filter_RAM_offset))
#define FDCAN1_RxFIFO0 ((CAN_RxBufferElement *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_RX_FIFO0_offset))
#define FDCAN1_RxFIFO1 ((CAN_RxBufferElement *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_RX_FIFO1_offset))
#define FDCAN1_RxBuffer ((CAN_RxBufferElement *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_RX_BUFFER_offset))
#define FDCAN1_TxEventFIFO ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_TX_EVENT_FIFO_offset))
#define FDCAN1_TxBuffer ((CAN_TxBufferElement *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_TX_BUFFER_offset))
#define FDCAN1_TriggerMemory ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_TRIGGER_MEMORY_offset))

#define FDCAN2_StandartMessageIDFilter ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_Std_Filter_RAM_offset))
#define FDCAN2_ExtendedMessageIDFilter ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_Ext_Filter_RAM_offset))
#define FDCAN2_RxFIFO0 ((CAN_RxBufferElement *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_RX_FIFO0_offset))
#define FDCAN2_RxFIFO1 ((CAN_RxBufferElement *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_RX_FIFO1_offset))
#define FDCAN2_RxBuffer ((CAN_RxBufferElement *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_RX_BUFFER_offset))
#define FDCAN2_TxEventFIFO ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_TX_EVENT_FIFO_offset))
#define FDCAN2_TxBuffer ((CAN_TxBufferElement *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_TX_BUFFER_offset))
#define FDCAN2_TriggerMemory ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_TRIGGER_MEMORY_offset))


// Register SPI4 --> checked!
struct spi {   
    volatile uint32_t CR1;          // 0x00
    volatile uint32_t CR2;          // 0x04  
    volatile uint32_t CFG1;         // 0x08
    volatile uint32_t CFG2;         // 0x0C
    volatile uint32_t IER;          // 0x10
    volatile uint32_t SR;           // 0x14
    volatile uint32_t IFCR;         // 0x18
    volatile uint32_t Reserved1;    // 0x1C
    volatile uint32_t TXDR;         // 0x20
    volatile uint32_t Reserved2;    // 0x24
    volatile uint32_t Reserved3;    // 0x28
    volatile uint32_t Reserved4;    // 0x2C
    volatile uint32_t RXDR;         // 0x30
    volatile uint32_t Reserved5;    // 0x34
    volatile uint32_t Reserved6;    // 0x38
    volatile uint32_t Reserved7;    // 0x3C  
    volatile uint32_t CRCPOLY;      // 0x40
    volatile uint32_t TXCRC;        // 0x44
    volatile uint32_t RXCRC;        // 0x48
    volatile uint32_t UDRDR;        // 0x4C
};
#define SPI4 ((struct spi *) 0x40013400) // checked!



// Register GPIO --> checked!
struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};
#define GPIOA ((struct gpio *) 0x58020000)
#define GPIOB ((struct gpio *) 0x58020400)
#define GPIOC ((struct gpio *) 0x58020800)
#define GPIOD ((struct gpio *) 0x58020C00)
#define GPIOE ((struct gpio *) 0x58021000)
#define GPIOF ((struct gpio *) 0x58021400)


// Register RTC 
struct rtc {
    volatile uint32_t TR;        // 0x00
    volatile uint32_t DR;        // 0x04  
    volatile uint32_t CR;        // 0x08
    volatile uint32_t ISR;       // 0x0C
    volatile uint32_t PRER;      // 0x10
    volatile uint32_t WUTR;      // 0x14
    volatile uint32_t reserved1; // 0x18
    volatile uint32_t ALRMAR;    // 0x1C
    volatile uint32_t ALRMBR;    // 0x20
    volatile uint32_t WPR;       // 0x24
    volatile uint32_t SSR;       // 0x28
    volatile uint32_t SHIFTR;    // 0x2C
    volatile uint32_t TSTR;      // 0x30
    volatile uint32_t TSDR;      // 0x34
    volatile uint32_t TSSSR;     // 0x38
    volatile uint32_t CALR;      // 0x3C
    volatile uint32_t TAMPCR;    // 0x40
    volatile uint32_t ALRMASSR;  // 0x44
    volatile uint32_t ALRMBSSR;  // 0x48
    volatile uint32_t OR;        // 0x4C
    volatile uint32_t BKP0R;     // 0x50
};
#define RTC ((struct rtc *) 0x58004000)

#endif