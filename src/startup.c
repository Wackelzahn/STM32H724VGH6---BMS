
// startup.c
#include <stdint.h>

// External symbols defined in link.ld or other files
extern long _sbss, _ebss, _sdata, _edata, _sidata;
extern void _estack(void);  // Stack top, defined in link.ld
extern int main(void);      // Main function from main.c

// Weak declarations for interrupt handlers (can be overridden elsewhere)
void SysTick_IRQHandler(void) __attribute__((weak));
void TIM16_FDCAN_IT0_IRQHandler(void) __attribute__((weak));
void TIM17_FDCAN_IT1_IRQHandler(void) __attribute__((weak));
void USART1_IRQHandler(void) __attribute__((weak));
void USART2_IRQHandler(void) __attribute__((weak));

// Reset handler: Initialize .bss and .data, then call main()
__attribute__((naked, noreturn)) void _reset(void) {
  // Zero out .bss section
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  // Copy .data section from flash to RAM
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop if main() returns
}

// Interrupt vector table
__attribute__((section(".vectors"))) void (*const tab[16 + 32])(void) = {
  _estack,           // 0: Stack pointer
  _reset,            // 1: Reset handler
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 2-14: Core exceptions
  SysTick_IRQHandler,   // 15: SysTick
  // IRQ0 to IRQ31 (positions 16 to 47)
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // IRQ0 .. IRQ9
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // IRQ10 .. IRQ19
  0,                            // IRQ20
  TIM16_FDCAN_IT0_IRQHandler,  // IRQ21: FDCAN Interrupt 0
  TIM17_FDCAN_IT1_IRQHandler,  // IRQ22: FDCAN Interrupt 1
  0, 0, 0, 0, 0, 0,      // IRQ23 .. IRQ26
  USART1_IRQHandler,      // IRQ27: USART1
  USART2_IRQHandler,      // IRQ28: USART2
  0           // IRQ29 .. IRQ31
};

// Default weak implementations (optional, can be overridden in other files)
__attribute__((weak)) void SysTick_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM16_FDCAN_IT0_IRQHandler(void) { while (1); }
__attribute__((weak)) void TIM17_FDCAN_IT1_IRQHandler(void) { while (1); }
__attribute__((weak)) void USART1_IRQHandler(void) { while (1); }
__attribute__((weak)) void USART2_IRQHandler(void) { while (1); }