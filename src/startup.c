
// startup.c for STM32H742
#include <stdint.h>

// External symbols defined in link.ld or other files
extern long _sbss, _ebss, _sdata, _edata, _sidata;
extern void _estack(void);  // Stack top, defined in link.ld
extern void _reset(void);   // Reset handler
extern int main(void);      // Main function from main.c

// Weak declarations for interrupt handlers (can be overridden elsewhere)
void NMI_Handler(void) __attribute__((weak));
void HardFault_Handler(void) __attribute__((weak));
void MemManage_Handler(void) __attribute__((weak));
void BusFault_Handler(void) __attribute__((weak));
void UsageFault_Handler(void) __attribute__((weak));
void SVCall_Handler(void) __attribute__((weak));
void DebugMon_Handler(void) __attribute__((weak));
void PendSV_Handler(void) __attribute__((weak));

void SysTick_IRQHandler(void) __attribute__((weak));
void WWDG1_IRQHandler(void) __attribute__((weak));
void PVD_PVM_IRQHandler(void) __attribute__((weak));
void RTC_TAMP_STAMP_CSS_LSE_IRQHandler(void) __attribute__((weak));
void RTC_WKUP_IRQHandler(void) __attribute__((weak));
void FLASH_IRQHandler(void) __attribute__((weak));
void RCC_IRQHandler(void) __attribute__((weak));
void EXTI0_IRQHandler(void) __attribute__((weak));
void EXTI1_IRQHandler(void) __attribute__((weak));
void EXTI2_IRQHandler(void) __attribute__((weak));
void EXTI3_IRQHandler(void) __attribute__((weak));
void EXTI4_IRQHandler(void) __attribute__((weak));
void DMA_STR0_IRQHandler(void) __attribute__((weak));
void DMA_STR1_IRQHandler(void) __attribute__((weak));
void DMA_STR2_IRQHandler(void) __attribute__((weak));
void DMA_STR3_IRQHandler(void) __attribute__((weak));
void DMA_STR4_IRQHandler(void) __attribute__((weak));
void DMA_STR5_IRQHandler(void) __attribute__((weak));
void DMA_STR6_IRQHandler(void) __attribute__((weak));
void ADC1_2_IRQHandler(void) __attribute__((weak));
void FDCAN1_IT0_IRQHandler(void) __attribute__((weak));
void FDCAN2_IT0_IRQHandler(void) __attribute__((weak));
void FDCAN1_IT1_IRQHandler(void) __attribute__((weak));
void FDCAN2_IT1_IRQHandler(void) __attribute__((weak));
void EXTI9_5_IRQHandler(void) __attribute__((weak));
void TIM1_BRK_IRQHandler(void) __attribute__((weak));
void TIM1_UP_IRQHandler(void) __attribute__((weak));
void TIM1_TRG_COM_IRQHandler(void) __attribute__((weak));
void TIM_CC_IRQHandler(void) __attribute__((weak));
void TIM2_IRQHandler(void) __attribute__((weak));
void TIM3_IRQHandler(void) __attribute__((weak));
void TIM4_IRQHandler(void) __attribute__((weak));
void I2C1_EV_IRQHandler(void) __attribute__((weak));
void I2C1_ER_IRQHandler(void) __attribute__((weak));
void I2C2_EV_IRQHandler(void) __attribute__((weak));
void I2C2_ER_IRQHandler(void) __attribute__((weak));
void SPI1_IRQHandler(void) __attribute__((weak));
void SPI2_IRQHandler(void) __attribute__((weak));
void USART1_IRQHandler(void) __attribute__((weak));
void USART2_IRQHandler(void) __attribute__((weak));
void USART3_IRQHandler(void) __attribute__((weak));
void EXTI15_10_IRQHandler(void) __attribute__((weak));
void RTC_ALARM_IRQHandler(void) __attribute__((weak));
void TIM8_BRK_TIM12_IRQHandler(void) __attribute__((weak));
void TIM8_UP_TIM13_IRQHandler(void) __attribute__((weak));
void TIM8_TRG_COM_TIM14_IRQHandler(void) __attribute__((weak));
void TIM8_CC_IRQHandler(void) __attribute__((weak));
void DMA1_STR7_IRQHandler(void) __attribute__((weak));
void FMC_IRQHandler(void) __attribute__((weak));
void SDMMC1_IRQHandler(void) __attribute__((weak));
void TIM5_IRQHandler(void) __attribute__((weak));
void SPI3_IRQHandler(void) __attribute__((weak));
void UART4_IRQHandler(void) __attribute__((weak));
void UART5_IRQHandler(void) __attribute__((weak));
void TIM6_DAC_IRQHandler(void) __attribute__((weak));
void TIM7_IRQHandler(void) __attribute__((weak));
void DMA2_STR0_IRQHandler(void) __attribute__((weak));
void DMA2_STR1_IRQHandler(void) __attribute__((weak));
void DMA2_STR2_IRQHandler(void) __attribute__((weak));
void DMA2_STR3_IRQHandler(void) __attribute__((weak));
void DMA2_STR4_IRQHandler(void) __attribute__((weak));
void ETH_IRQHandler(void) __attribute__((weak));
void ETH_WKUP_IRQHandler(void) __attribute__((weak));
void FDCAN_CAL_IRQHandler(void) __attribute__((weak));
void DMA2_STR5_IRQHandler(void) __attribute__((weak));
void DMA2_STR6_IRQHandler(void) __attribute__((weak));
void DMA2_STR7_IRQHandler(void) __attribute__((weak));
void USART6_IRQHandler(void) __attribute__((weak));
void I2C3_EV_IRQHandler(void) __attribute__((weak));
void I2C3_ER_IRQHandler(void) __attribute__((weak));
void OTG_HS_EP1_OUT_IRQHandler(void) __attribute__((weak));
void OTG_HS_EP1_IN_IRQHandler(void) __attribute__((weak));
void OTG_HS_WKUP_IRQHandler(void) __attribute__((weak));
void OTG_HS_IRQHandler(void) __attribute__((weak));
void DCMI_IRQHandler(void) __attribute__((weak));
void CRYP_IRQHandler(void) __attribute__((weak));
void HASH_RNG_IRQHandler(void) __attribute__((weak));
void FPU_IRQHandler(void) __attribute__((weak));
void UART7_IRQHandler(void) __attribute__((weak));
void UART8_IRQHandler(void) __attribute__((weak));
void SPI4_IRQHandler(void) __attribute__((weak));
void SPI5_IRQHandler(void) __attribute__((weak));
void SPI6_IRQHandler(void) __attribute__((weak));
void SAI1_IRQHandler(void) __attribute__((weak));
void LTDC_IRQHandler(void) __attribute__((weak));
void LTDC_ER_IRQHandler(void) __attribute__((weak));
void DMA2D_IRQHandler(void) __attribute__((weak));
void SAI2_IRQHandler(void) __attribute__((weak));
void QUADSPI_IRQHandler(void) __attribute__((weak));
void LPTIM1_IRQHandler(void) __attribute__((weak));
void CEC_IRQHandler(void) __attribute__((weak));
void I2C4_EV_IRQHandler(void) __attribute__((weak));
void I2C4_ER_IRQHandler(void) __attribute__((weak));
void SPDIF_IRQHandler(void) __attribute__((weak));
void OTG_FS_EP1_OUT_IRQHandler(void) __attribute__((weak));
void OTG_FS_EP1_IN_IRQHandler(void) __attribute__((weak));
void OTG_FS_WKUP_IRQHandler(void) __attribute__((weak));
void OTG_FS_IRQHandler(void) __attribute__((weak));
void DMAMUX1_OV_IRQHandler(void) __attribute__((weak));
void HRTIM1_MST_IRQHandler(void) __attribute__((weak));
void HRTIM1_TIMA_IRQHandler(void) __attribute__((weak));
void HRTIM_TIMB_IRQHandler(void) __attribute__((weak));
void HRTIM1_TIMC_IRQHandler(void) __attribute__((weak));
void HRTIM1_TIMD_IRQHandler(void) __attribute__((weak));
void HRTIM_TIME_IRQHandler(void) __attribute__((weak));
void HRTIM1_FLT_IRQHandler(void) __attribute__((weak));
void DFSDM1_FLT0_IRQHandler(void) __attribute__((weak));
void DFSDM1_FLT1_IRQHandler(void) __attribute__((weak));
void DFSDM1_FLT2_IRQHandler(void) __attribute__((weak));
void DFSDM1_FLT3_IRQHandler(void) __attribute__((weak));
void SAI3_IRQHandler(void) __attribute__((weak));
void SWPMI1_IRQHandler(void) __attribute__((weak));
void TIM15_IRQHandler(void) __attribute__((weak));
void TIM16_IRQHandler(void) __attribute__((weak));
void TIM17_IRQHandler(void) __attribute__((weak));
void MDIOS_WKUP_IRQHandler(void) __attribute__((weak));
void MDIOS_IRQHandler(void) __attribute__((weak));
void JPEG_IRQHandler(void) __attribute__((weak));
void MDMA_IRQHandler(void) __attribute__((weak));
void SDMMC2_IRQHandler(void) __attribute__((weak));
void HSEM0_IRQHandler(void) __attribute__((weak));
void ADC3_IRQHandler(void) __attribute__((weak));
void DMAMUX2_OVR_IRQHandler(void) __attribute__((weak));
void BDMA_CH0_IRQHandler(void) __attribute__((weak));
void BDMA_CH1_IRQHandler(void) __attribute__((weak));
void BDMA_CH2_IRQHandler(void) __attribute__((weak));
void BDMA_CH3_IRQHandler(void) __attribute__((weak));
void BDMA_CH4_IRQHandler(void) __attribute__((weak));
void BDMA_CH5_IRQHandler(void) __attribute__((weak));
void BDMA_CH6_IRQHandler(void) __attribute__((weak));
void BDMA_CH7_IRQHandler(void) __attribute__((weak));
void COMP_IRQHandler(void) __attribute__((weak));
void LPTIM2_IRQHandler(void) __attribute__((weak));
void LPTIM3_IRQHandler(void) __attribute__((weak));
void LPTIM4_IRQHandler(void) __attribute__((weak));
void LPTIM5_IRQHandler(void) __attribute__((weak));
void LPUART_IRQHandler(void) __attribute__((weak));
void WWDG1_RST_IRQHandler(void) __attribute__((weak));
void CRS_IRQHandler(void) __attribute__((weak));
void RAMECC1_IRQHandler(void) __attribute__((weak));
void SAI4_IRQHandler(void) __attribute__((weak));
void WKUP_IRQHandler(void) __attribute__((weak));

// Reset handler: Initialize .bss and .data, then call main()
__attribute__((naked, noreturn)) void _reset(void) {
  // Zero out .bss section
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  // Copy .data section from flash to RAM
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;
  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop if main() returns
}

// Interrupt vector table for STM32H742 (exactly as per RM0433)
__attribute__((section(".vectors"))) void (*const tab[16 + 150])(void) = {
    _estack,                          // 0: Stack pointer
    _reset,                           // 1: Reset handler
    NMI_Handler,                      // 2: NMI
    HardFault_Handler,                // 3: HardFault
    MemManage_Handler,                // 4: MemManage
    BusFault_Handler,                 // 5: BusFault
    UsageFault_Handler,               // 6: UsageFault
    0, 0, 0, 0,                      // 7-10: Reserved
    SVCall_Handler,                   // 11: SVCall
    DebugMon_Handler,                 // 12: Debug Monitor
    0,                                // 13: Reserved
    PendSV_Handler,                   // 14: PendSV
    SysTick_IRQHandler,               // 15: SysTick
  
  // External interrupts (IRQ0-IRQ149) - positions match RM0433 Table 145
  WWDG1_IRQHandler,                 // 16 (IRQ0): Window Watchdog
  PVD_PVM_IRQHandler,               // 17 (IRQ1): PVD through EXTI
  RTC_TAMP_STAMP_CSS_LSE_IRQHandler,// 18 (IRQ2): RTC tamper, timestamp
  RTC_WKUP_IRQHandler,              // 19 (IRQ3): RTC Wakeup
  FLASH_IRQHandler,                 // 20 (IRQ4): FLASH
  RCC_IRQHandler,                   // 21 (IRQ5): RCC
  EXTI0_IRQHandler,                 // 22 (IRQ6): EXTI Line 0
  EXTI1_IRQHandler,                 // 23 (IRQ7): EXTI Line 1
  EXTI2_IRQHandler,                 // 24 (IRQ8): EXTI Line 2
  EXTI3_IRQHandler,                 // 25 (IRQ9): EXTI Line 3
  EXTI4_IRQHandler,                 // 26 (IRQ10): EXTI Line 4
  DMA_STR0_IRQHandler,              // 27 (IRQ11): DMA1 Stream0
  DMA_STR1_IRQHandler,              // 28 (IRQ12): DMA1 Stream1
  DMA_STR2_IRQHandler,              // 29 (IRQ13): DMA1 Stream2
  DMA_STR3_IRQHandler,              // 30 (IRQ14): DMA1 Stream3
  DMA_STR4_IRQHandler,              // 31 (IRQ15): DMA1 Stream4
  DMA_STR5_IRQHandler,              // 32 (IRQ16): DMA1 Stream5
  DMA_STR6_IRQHandler,              // 33 (IRQ17): DMA1 Stream6
  ADC1_2_IRQHandler,                // 34 (IRQ18): ADC1 and ADC2
  FDCAN1_IT0_IRQHandler,            // 35 (IRQ19): FDCAN1 Interrupt 0
  FDCAN2_IT0_IRQHandler,            // 36 (IRQ20): FDCAN2 Interrupt 0
  FDCAN1_IT1_IRQHandler,            // 37 (IRQ21): FDCAN1 Interrupt 1
  FDCAN2_IT1_IRQHandler,            // 38 (IRQ22): FDCAN2 Interrupt 1
  EXTI9_5_IRQHandler,               // 39 (IRQ23): EXTI Line[9:5]
  TIM1_BRK_IRQHandler,              // 40 (IRQ24): TIM1 Break
  TIM1_UP_IRQHandler,               // 41 (IRQ25): TIM1 Update
  TIM1_TRG_COM_IRQHandler,          // 42 (IRQ26): TIM1 Trigger and Commutation
  TIM_CC_IRQHandler,                // 43 (IRQ27): TIM1 Capture Compare
  TIM2_IRQHandler,                  // 44 (IRQ28): TIM2
  TIM3_IRQHandler,                  // 45 (IRQ29): TIM3
  TIM4_IRQHandler,                  // 46 (IRQ30): TIM4
  I2C1_EV_IRQHandler,               // 47 (IRQ31): I2C1 Event
  I2C1_ER_IRQHandler,               // 48 (IRQ32): I2C1 Error
  I2C2_EV_IRQHandler,               // 49 (IRQ33): I2C2 Event
  I2C2_ER_IRQHandler,               // 50 (IRQ34): I2C2 Error
  SPI1_IRQHandler,                  // 51 (IRQ35): SPI1
  SPI2_IRQHandler,                  // 52 (IRQ36): SPI2
  USART1_IRQHandler,                // 53 (IRQ37): USART1
  USART2_IRQHandler,                // 54 (IRQ38): USART2
  USART3_IRQHandler,                // 55 (IRQ39): USART3
  EXTI15_10_IRQHandler,             // 56 (IRQ40): EXTI Line[15:10]
  RTC_ALARM_IRQHandler,             // 57 (IRQ41): RTC Alarms through EXTI
  0,                                // 58 (IRQ42): Reserved
  TIM8_BRK_TIM12_IRQHandler,        // 59 (IRQ43): TIM8 Break and TIM12
  TIM8_UP_TIM13_IRQHandler,         // 60 (IRQ44): TIM8 Update and TIM13
  TIM8_TRG_COM_TIM14_IRQHandler,    // 61 (IRQ45): TIM8 Trigger/Commutation and TIM14
  TIM8_CC_IRQHandler,               // 62 (IRQ46): TIM8 Capture Compare
  DMA1_STR7_IRQHandler,             // 63 (IRQ47): DMA1 Stream7
  FMC_IRQHandler,                   // 64 (IRQ48): FMC
  SDMMC1_IRQHandler,                // 65 (IRQ49): SDMMC1
  TIM5_IRQHandler,                  // 66 (IRQ50): TIM5
  SPI3_IRQHandler,                  // 67 (IRQ51): SPI3
  UART4_IRQHandler,                 // 68 (IRQ52): UART4
  UART5_IRQHandler,                 // 69 (IRQ53): UART5
  TIM6_DAC_IRQHandler,              // 70 (IRQ54): TIM6 and DAC underrun
  TIM7_IRQHandler,                  // 71 (IRQ55): TIM7
  DMA2_STR0_IRQHandler,             // 72 (IRQ56): DMA2 Stream0
  DMA2_STR1_IRQHandler,             // 73 (IRQ57): DMA2 Stream1
  DMA2_STR2_IRQHandler,             // 74 (IRQ58): DMA2 Stream2
  DMA2_STR3_IRQHandler,             // 75 (IRQ59): DMA2 Stream3
  DMA2_STR4_IRQHandler,             // 76 (IRQ60): DMA2 Stream4
  ETH_IRQHandler,                   // 77 (IRQ61): Ethernet
  ETH_WKUP_IRQHandler,              // 78 (IRQ62): Ethernet Wakeup
  FDCAN_CAL_IRQHandler,             // 79 (IRQ63): FDCAN calibration
  0,                                // 80 (IRQ64): Cortex-M7 Send event (reserved)
  0, 0, 0,                          // 81-83 (IRQ65-67): Reserved
  DMA2_STR5_IRQHandler,             // 84 (IRQ68): DMA2 Stream5
  DMA2_STR6_IRQHandler,             // 85 (IRQ69): DMA2 Stream6
  DMA2_STR7_IRQHandler,             // 86 (IRQ70): DMA2 Stream7
  USART6_IRQHandler,                // 87 (IRQ71): USART6
  I2C3_EV_IRQHandler,               // 88 (IRQ72): I2C3 Event
  I2C3_ER_IRQHandler,               // 89 (IRQ73): I2C3 Error
  OTG_HS_EP1_OUT_IRQHandler,        // 90 (IRQ74): USB OTG HS EP1 OUT
  OTG_HS_EP1_IN_IRQHandler,         // 91 (IRQ75): USB OTG HS EP1 IN
  OTG_HS_WKUP_IRQHandler,           // 92 (IRQ76): USB OTG HS Wakeup
  OTG_HS_IRQHandler,                // 93 (IRQ77): USB OTG HS
  DCMI_IRQHandler,                  // 94 (IRQ78): DCMI
  CRYP_IRQHandler,                  // 95 (IRQ79): CRYP
  HASH_RNG_IRQHandler,              // 96 (IRQ80): HASH and RNG
  FPU_IRQHandler,                   // 97 (IRQ81): FPU
  UART7_IRQHandler,                 // 98 (IRQ82): UART7
  UART8_IRQHandler,                 // 99 (IRQ83): UART8
  SPI4_IRQHandler,                  // 100 (IRQ84): SPI4
  SPI5_IRQHandler,                  // 101 (IRQ85): SPI5
  SPI6_IRQHandler,                  // 102 (IRQ86): SPI6
  SAI1_IRQHandler,                  // 103 (IRQ87): SAI1
  LTDC_IRQHandler,                  // 104 (IRQ88): LTDC
  LTDC_ER_IRQHandler,               // 105 (IRQ89): LTDC error
  DMA2D_IRQHandler,                 // 106 (IRQ90): DMA2D
  SAI2_IRQHandler,                  // 107 (IRQ91): SAI2
  QUADSPI_IRQHandler,               // 108 (IRQ92): QUADSPI
  LPTIM1_IRQHandler,                // 109 (IRQ93): LPTIM1
  CEC_IRQHandler,                   // 110 (IRQ94): HDMI-CEC
  I2C4_EV_IRQHandler,               // 111 (IRQ95): I2C4 Event
  I2C4_ER_IRQHandler,               // 112 (IRQ96): I2C4 Error
  SPDIF_IRQHandler,                 // 113 (IRQ97): SPDIFRX
  OTG_FS_EP1_OUT_IRQHandler,        // 114 (IRQ98): USB OTG FS EP1 OUT
  OTG_FS_EP1_IN_IRQHandler,         // 115 (IRQ99): USB OTG FS EP1 IN
  OTG_FS_WKUP_IRQHandler,           // 116 (IRQ100): USB OTG FS Wakeup
  OTG_FS_IRQHandler,                // 117 (IRQ101): USB OTG FS
  DMAMUX1_OV_IRQHandler,            // 118 (IRQ102): DMAMUX1 Overrun
  HRTIM1_MST_IRQHandler,            // 119 (IRQ103): HRTIM1 Master Timer
  HRTIM1_TIMA_IRQHandler,           // 120 (IRQ104): HRTIM1 Timer A
  HRTIM_TIMB_IRQHandler,            // 121 (IRQ105): HRTIM1 Timer B
  HRTIM1_TIMC_IRQHandler,           // 122 (IRQ106): HRTIM1 Timer C
  HRTIM1_TIMD_IRQHandler,           // 123 (IRQ107): HRTIM1 Timer D
  HRTIM_TIME_IRQHandler,            // 124 (IRQ108): HRTIM1 Timer E
  HRTIM1_FLT_IRQHandler,            // 125 (IRQ109): HRTIM1 Fault
  DFSDM1_FLT0_IRQHandler,           // 126 (IRQ110): DFSDM1 Filter0
  DFSDM1_FLT1_IRQHandler,           // 127 (IRQ111): DFSDM1 Filter1
  DFSDM1_FLT2_IRQHandler,           // 128 (IRQ112): DFSDM1 Filter2
  DFSDM1_FLT3_IRQHandler,           // 129 (IRQ113): DFSDM1 Filter3
  SAI3_IRQHandler,                  // 130 (IRQ114): SAI3
  SWPMI1_IRQHandler,                // 131 (IRQ115): SWPMI1
  TIM15_IRQHandler,                 // 132 (IRQ116): TIM15
  TIM16_IRQHandler,                 // 133 (IRQ117): TIM16
  TIM17_IRQHandler,                 // 134 (IRQ118): TIM17
  MDIOS_WKUP_IRQHandler,            // 135 (IRQ119): MDIOS Wakeup
  MDIOS_IRQHandler,                 // 136 (IRQ120): MDIOS
  JPEG_IRQHandler,                  // 137 (IRQ121): JPEG
  MDMA_IRQHandler,                  // 138 (IRQ122): MDMA
  0,                                // 139 (IRQ123): Reserved
  SDMMC2_IRQHandler,                // 140 (IRQ124): SDMMC2
  HSEM0_IRQHandler,                 // 141 (IRQ125): HSEM
  0,                                // 142 (IRQ126): Reserved
  ADC3_IRQHandler,                  // 143 (IRQ127): ADC3
  DMAMUX2_OVR_IRQHandler,           // 144 (IRQ128): DMAMUX2 Overrun
  BDMA_CH0_IRQHandler,              // 145 (IRQ129): BDMA Channel 0
  BDMA_CH1_IRQHandler,              // 146 (IRQ130): BDMA Channel 1
  BDMA_CH2_IRQHandler,              // 147 (IRQ131): BDMA Channel 2
  BDMA_CH3_IRQHandler,              // 148 (IRQ132): BDMA Channel 3
  BDMA_CH4_IRQHandler,              // 149 (IRQ133): BDMA Channel 4
  BDMA_CH5_IRQHandler,              // 150 (IRQ134): BDMA Channel 5
  BDMA_CH6_IRQHandler,              // 151 (IRQ135): BDMA Channel 6
  BDMA_CH7_IRQHandler,              // 152 (IRQ136): BDMA Channel 7
  COMP_IRQHandler,                  // 153 (IRQ137): COMP1 and COMP2
  LPTIM2_IRQHandler,                // 154 (IRQ138): LPTIM2
  LPTIM3_IRQHandler,                // 155 (IRQ139): LPTIM3
  LPTIM4_IRQHandler,                // 156 (IRQ140): LPTIM4
  LPTIM5_IRQHandler,                // 157 (IRQ141): LPTIM5
  LPUART_IRQHandler,                // 158 (IRQ142): LPUART
  WWDG1_RST_IRQHandler,             // 159 (IRQ143): WWDG1 RST
  CRS_IRQHandler,                   // 160 (IRQ144): CRS
  RAMECC1_IRQHandler,               // 161 (IRQ145): RAMECC
  SAI4_IRQHandler,                  // 162 (IRQ146): SAI4
  0, 0,                             // 163-164 (IRQ147-148): Reserved
  WKUP_IRQHandler,                  // 165 (IRQ149): WKUP0 to WKUP5 pins
};

// Default weak implementations (you can override these in other files)
__attribute__((weak)) void NMI_Handler(void) { while (1); }
__attribute__((weak)) void HardFault_Handler(void) { while (1); } // Trap for debugging
__attribute__((weak)) void MemManage_Handler(void) { while (1); }
__attribute__((weak)) void BusFault_Handler(void) { while (1); }
__attribute__((weak)) void UsageFault_Handler(void) { while (1); }
__attribute__((weak)) void SVCall_Handler(void) { while (1); }
__attribute__((weak)) void DebugMon_Handler(void) { while (1); }
__attribute__((weak)) void PendSV_Handler(void) { while (1); }
__attribute__((weak)) void SysTick_IRQHandler(void) { while (1); }
__attribute__((weak)) void WWDG1_IRQHandler(void) { while (1); }
__attribute__((weak)) void PVD_PVM_IRQHandler(void) { while (1); }
__attribute__((weak)) void RTC_TAMP_STAMP_CSS_LSE_IRQHandler(void) { while (1); }
__attribute__((weak)) void RTC_WKUP_IRQHandler(void) { while (1); }
__attribute__((weak)) void FLASH_IRQHandler(void) { while (1); }
__attribute__((weak)) void RCC_IRQHandler(void) { while (1); }
__attribute__((weak)) void EXTI0_IRQHandler(void) { while (1); }
__attribute__((weak)) void EXTI1_IRQHandler(void) { while (1); }
__attribute__((weak)) void EXTI2_IRQHandler(void) { while (1); }
__attribute__((weak)) void EXTI3_IRQHandler(void) { while (1); }
__attribute__((weak)) void EXTI4_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA_STR0_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA_STR1_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA_STR2_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA_STR3_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA_STR4_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA_STR5_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA_STR6_IRQHandler(void) { while (1); }
__attribute__((weak)) void ADC1_2_IRQHandler(void) { while (1); }
__attribute__((weak)) void FDCAN1_IT0_IRQHandler(void) { while (1); }
__attribute__((weak)) void FDCAN2_IT0_IRQHandler(void) { while (1); }
__attribute__((weak)) void FDCAN1_IT1_IRQHandler(void) { while (1); }
__attribute__((weak)) void FDCAN2_IT1_IRQHandler(void) { while (1); }
__attribute__((weak)) void EXTI9_5_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM1_BRK_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM1_UP_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM1_TRG_COM_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM_CC_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM2_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM3_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM4_IRQHandler(void) { while (1); }
__attribute__((weak)) void I2C1_EV_IRQHandler(void) { while (1); }
__attribute__((weak)) void I2C1_ER_IRQHandler(void) { while (1); }
__attribute__((weak)) void I2C2_EV_IRQHandler(void) { while (1); }
__attribute__((weak)) void I2C2_ER_IRQHandler(void) { while (1); }
__attribute__((weak)) void SPI1_IRQHandler(void) { while (1); }
__attribute__((weak)) void SPI2_IRQHandler(void) { while (1); }
__attribute__((weak)) void USART1_IRQHandler(void) { while (1); }
__attribute__((weak)) void USART2_IRQHandler(void) { while (1); }
__attribute__((weak)) void USART3_IRQHandler(void) { while (1); }
__attribute__((weak)) void EXTI15_10_IRQHandler(void) { while (1); }
__attribute__((weak)) void RTC_ALARM_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM8_BRK_TIM12_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM8_UP_TIM13_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM8_TRG_COM_TIM14_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM8_CC_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA1_STR7_IRQHandler(void) { while (1); }
__attribute__((weak)) void FMC_IRQHandler(void) { while (1); }
__attribute__((weak)) void SDMMC1_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM5_IRQHandler(void) { while (1); }
__attribute__((weak)) void SPI3_IRQHandler(void) { while (1); }
__attribute__((weak)) void UART4_IRQHandler(void) { while (1); }
__attribute__((weak)) void UART5_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM6_DAC_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM7_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA2_STR0_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA2_STR1_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA2_STR2_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA2_STR3_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA2_STR4_IRQHandler(void) { while (1); }
__attribute__((weak)) void ETH_IRQHandler(void) { while (1); }
__attribute__((weak)) void ETH_WKUP_IRQHandler(void) { while (1); }
__attribute__((weak)) void FDCAN_CAL_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA2_STR5_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA2_STR6_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA2_STR7_IRQHandler(void) { while (1); }
__attribute__((weak)) void USART6_IRQHandler(void) { while (1); }
__attribute__((weak)) void I2C3_EV_IRQHandler(void) { while (1); }
__attribute__((weak)) void I2C3_ER_IRQHandler(void) { while (1); }
__attribute__((weak)) void OTG_HS_EP1_OUT_IRQHandler(void) { while (1); }
__attribute__((weak)) void OTG_HS_EP1_IN_IRQHandler(void) { while (1); }
__attribute__((weak)) void OTG_HS_WKUP_IRQHandler(void) { while (1); }
__attribute__((weak)) void OTG_HS_IRQHandler(void) { while (1); }
__attribute__((weak)) void DCMI_IRQHandler(void) { while (1); }
__attribute__((weak)) void CRYP_IRQHandler(void) { while (1); }
__attribute__((weak)) void HASH_RNG_IRQHandler(void) { while (1); }
__attribute__((weak)) void FPU_IRQHandler(void) { while (1); }
__attribute__((weak)) void UART7_IRQHandler(void) { while (1); }
__attribute__((weak)) void UART8_IRQHandler(void) { while (1); }
__attribute__((weak)) void SPI4_IRQHandler(void) { while (1); }
__attribute__((weak)) void SPI5_IRQHandler(void) { while (1); }
__attribute__((weak)) void SPI6_IRQHandler(void) { while (1); }
__attribute__((weak)) void SAI1_IRQHandler(void) { while (1); }
__attribute__((weak)) void LTDC_IRQHandler(void) { while (1); }
__attribute__((weak)) void LTDC_ER_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMA2D_IRQHandler(void) { while (1); }
__attribute__((weak)) void SAI2_IRQHandler(void) { while (1); }
__attribute__((weak)) void QUADSPI_IRQHandler(void) { while (1); }
__attribute__((weak)) void LPTIM1_IRQHandler(void) { while (1); }
__attribute__((weak)) void CEC_IRQHandler(void) { while (1); }
__attribute__((weak)) void I2C4_EV_IRQHandler(void) { while (1); }
__attribute__((weak)) void I2C4_ER_IRQHandler(void) { while (1); }
__attribute__((weak)) void SPDIF_IRQHandler(void) { while (1); }
__attribute__((weak)) void OTG_FS_EP1_OUT_IRQHandler(void) { while (1); }
__attribute__((weak)) void OTG_FS_EP1_IN_IRQHandler(void) { while (1); }
__attribute__((weak)) void OTG_FS_WKUP_IRQHandler(void) { while (1); }
__attribute__((weak)) void OTG_FS_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMAMUX1_OV_IRQHandler(void) { while (1); }
__attribute__((weak)) void HRTIM1_MST_IRQHandler(void) { while (1); }
__attribute__((weak)) void HRTIM1_TIMA_IRQHandler(void) { while (1); }
__attribute__((weak)) void HRTIM_TIMB_IRQHandler(void) { while (1); }
__attribute__((weak)) void HRTIM1_TIMC_IRQHandler(void) { while (1); }
__attribute__((weak)) void HRTIM1_TIMD_IRQHandler(void) { while (1); }
__attribute__((weak)) void HRTIM_TIME_IRQHandler(void) { while (1); }
__attribute__((weak)) void HRTIM1_FLT_IRQHandler(void) { while (1); }
__attribute__((weak)) void DFSDM1_FLT0_IRQHandler(void) { while (1); }
__attribute__((weak)) void DFSDM1_FLT1_IRQHandler(void) { while (1); }
__attribute__((weak)) void DFSDM1_FLT2_IRQHandler(void) { while (1); }
__attribute__((weak)) void DFSDM1_FLT3_IRQHandler(void) { while (1); }
__attribute__((weak)) void SAI3_IRQHandler(void) { while (1); }
__attribute__((weak)) void SWPMI1_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM15_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM16_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM17_IRQHandler(void) { while (1); }
__attribute__((weak)) void MDIOS_WKUP_IRQHandler(void) { while (1); }
__attribute__((weak)) void MDIOS_IRQHandler(void) { while (1); }
__attribute__((weak)) void JPEG_IRQHandler(void) { while (1); }
__attribute__((weak)) void MDMA_IRQHandler(void) { while (1); }
__attribute__((weak)) void SDMMC2_IRQHandler(void) { while (1); }
__attribute__((weak)) void HSEM0_IRQHandler(void) { while (1); }
__attribute__((weak)) void ADC3_IRQHandler(void) { while (1); }
__attribute__((weak)) void DMAMUX2_OVR_IRQHandler(void) { while (1); }
__attribute__((weak)) void BDMA_CH0_IRQHandler(void) { while (1); }
__attribute__((weak)) void BDMA_CH1_IRQHandler(void) { while (1); }
__attribute__((weak)) void BDMA_CH2_IRQHandler(void) { while (1); }
__attribute__((weak)) void BDMA_CH3_IRQHandler(void) { while (1); }
__attribute__((weak)) void BDMA_CH4_IRQHandler(void) { while (1); }
__attribute__((weak)) void BDMA_CH5_IRQHandler(void) { while (1); }
__attribute__((weak)) void BDMA_CH6_IRQHandler(void) { while (1); }
__attribute__((weak)) void BDMA_CH7_IRQHandler(void) { while (1); }
__attribute__((weak)) void COMP_IRQHandler(void) { while (1); }
__attribute__((weak)) void LPTIM2_IRQHandler(void) { while (1); }
__attribute__((weak)) void LPTIM3_IRQHandler(void) { while (1); }
__attribute__((weak)) void LPTIM4_IRQHandler(void) { while (1); }
__attribute__((weak)) void LPTIM5_IRQHandler(void) { while (1); }
__attribute__((weak)) void LPUART_IRQHandler(void) { while (1); }
__attribute__((weak)) void WWDG1_RST_IRQHandler(void) { while (1); }
__attribute__((weak)) void CRS_IRQHandler(void) { while (1); }
__attribute__((weak)) void RAMECC1_IRQHandler(void) { while (1); }
__attribute__((weak)) void SAI4_IRQHandler(void) { while (1); }
__attribute__((weak)) void WKUP_IRQHandler(void) { while (1); }