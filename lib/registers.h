#ifndef REGISTERS_H
#define REGISTERS_H

#include <inttypes.h>



// Register RCC (Reset and Clock Control)
struct rcc {
  volatile uint32_t CR, ICSCR, CFGR, PLLCFGR, RESERVED0, CRRCR, CIER, CIFR,
      CICR, IOPRSTR, AHBRSTR, APBRSTR1, APBRSTR2, IOPENR, AHBENR,
      APBENR1, APBENR2, IOPSMENR, AHBSMENR, APBSMENR1, APBSMENR2,
      CCIPR, CCIPR2, BDCR, CSR;
};
#define RCC ((struct rcc *) 0x40021000)



// Register I2C
struct I2C {
  volatile uint32_t CR1, CR2, OAR1, OAR2, TIMINGR, TIMEOUTR, ISR, ICR, PECR,
      RXDR, TXDR;
};
#define I2C1 ((struct I2C *) 0x40005400)
#define I2C2 ((struct I2C *) 0x40005800)



// Register SYSTICK
struct systick {
  volatile uint32_t CSR, RVR, CVR, CALIB;
};
#define SYST ((struct systick *) 0xE000E010)



// Register NVIC
struct nvic {
  volatile uint32_t ISER[2];  // Interrupt Set-Enable Register (32 IRQs)
  uint32_t RESERVED0[30];
  volatile uint32_t ICER[2];  // Interrupt Clear-Enable Register
  uint32_t RESERVED1[30];
  volatile uint32_t ISPR[2];  // Interrupt Set-Pending Register
  uint32_t RESERVED2[30];
  volatile uint32_t ICPR[2];  // Interrupt Clear-Pending Register
  uint32_t RESERVED3[30];
  volatile uint32_t IPR[8];   // Interrupt Priority Registers (32 IRQs, 4 bits each)
};
#define NVIC ((struct nvic *) 0xE000E100)



// Register SCB
struct scb {
  volatile uint32_t CPUID, ICSR, VTOR, AIRCR, SCR, CCR, SHPR[2], SHCSR,
      CFSR, HFSR, DFSR, MMFAR, BFAR, AFSR;
};
#define SCB ((struct scb *) 0xE000ED00)



// Register USART
struct usart {
  volatile uint32_t CR1, CR2, CR3, BRR, GTPR, RTOR, RQR, ISR, ICR, RDR, TDR,
      PRESC;
};
#define USART1 ((struct usart *) 0x40013800)
#define USART2 ((struct usart *) 0x40004400)





// FD CAN register
struct fdcan {
  volatile uint32_t CREL, ENDN, RESERVED0, DBTP, TEST, RWD, CCCR, NBTP, TSCC, TSCV, TOCC,
      TOCV, RESERVED1[4], ECR, PSR, TDCR, RESERVED2, IR, IE, ILS, ILE, RESERVED3[8], RXGFC, XIDAM,
      HPMS, RESERVEDX, RXF0S, RXF0A, RXF1S, RXF1A, RESERVED4[8], TXBC, TXFQS, TXBRP, TXBAR, TXBCR, TXBTO, TXBCF, TXBTIE,
      TXBCIE, TXEFS, TXEFA, CKDIV;
};
#define FDCAN1 ((struct fdcan *) 0x40006400)
#define FDCAN2 ((struct fdcan *) 0x40006800)


typedef struct {
  uint32_t T0;
  uint32_t T1;
  uint8_t data[8];
} CAN_TxBufferElement;

// FDCAN Receive Message RAM structure , only 8 bytes of data are used. 
typedef struct {
  uint32_t R0;
  uint32_t R1;
  uint8_t data[8];
  uint32_t reserved[14]; // 2 reserved bytes
} CAN_RxBufferElement;


// FDCAN Message RAM base address
#define FDCAN_MESSAGE_RAM_BASE    0x4000B400
#define FDCAN1_MESSAGE_RAM_BASE_offset 0x0000
#define FDCAN2_MESSAGE_RAM_BASE_offset 0x0350
#define FDCAN_Std_Filter_RAM_offset 0x0000
#define FDCAN_Ext_Filter_RAM_offset 0x0070
#define FDCAN_RX_FIFO0_offset 0x00B0
#define FDCAN_RX_FIFO1_offset 0x0188
#define FDCAN_TX_EVENT_FIFO_offset 0x0260
#define FDCAN_TX_BUFFER_offset 0x0278

#define FDCAN1_StandartMessageIDFilter ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_Std_Filter_RAM_offset))
#define FDCAN1_ExtendedMessageIDFilter ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_Ext_Filter_RAM_offset))
#define FDCAN1_RxFIFO0 ((CAN_RxBufferElement *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_RX_FIFO0_offset))
#define FDCAN1_RxFIFO1 ((CAN_RxBufferElement *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_RX_FIFO1_offset))
#define FDCAN1_TxEventFIFO ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_TX_EVENT_FIFO_offset))
#define FDCAN1_TxBuffer ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_TX_BUFFER_offset))

#define FDCAN2_StandartMessageIDFilter ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_Std_Filter_RAM_offset))
#define FDCAN2_ExtendedMessageIDFilter ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_Ext_Filter_RAM_offset))
#define FDCAN2_RxFIFO0 ((CAN_RxBufferElement *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_RX_FIFO0_offset))
#define FDCAN2_RxFIFO1 ((CAN_RxBufferElement *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_RX_FIFO1_offset))
#define FDCAN2_TxEventFIFO ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_TX_EVENT_FIFO_offset))
#define FDCAN2_TxBuffer ((uint32_t *) (FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset + FDCAN_TX_BUFFER_offset))










/* // FDCAN Message RAM structure
struct FDCAN_MessageRAM {
  volatile uint32_t TxEventFIFO[2]; // Tx Event FIFO
  volatile uint32_t TxBuffer[16];   // Tx Buffer
  volatile uint32_t RxFIFO[16];     // Rx FIFO
  volatile uint32_t RxBuffer[16];   // Rx Buffer
  volatile uint32_t RxEventFIFO[2]; // Rx Event FIFO
};
#define FDCAN_MESSAGE_RAM_BASE ((struct FDCAN_MessageRAM *) FDCAN_MESSAGE_RAM) */





// Register GPIO
struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2], BRR;
};
#define GPIOA ((struct gpio *) 0x50000000)
#define GPIOB ((struct gpio *) 0x50000400)
#define GPIOC ((struct gpio *) 0x50000800)
#define GPIOD ((struct gpio *) 0x50000C00)
#define GPIOE ((struct gpio *) 0x50001000)
#define GPIOF ((struct gpio *) 0x50001400)





#endif