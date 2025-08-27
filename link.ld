
ENTRY(_reset);
MEMORY {    /* Specific for STM32H742VGH6 */
  flash(rx)  : ORIGIN = 0x08000000, LENGTH = 1024k /* Flash memory for STM32H742VGH6 */
  dtcm(rwx)    : ORIGIN = 0x20000000, LENGTH = 128k    /* DTCM RAM */
  axi_sram(rwx): ORIGIN = 0x24000000, LENGTH = 384k    /* AXI SRAM */
  sram1(rwx)   : ORIGIN = 0x30000000, LENGTH = 32k     /* AHB SRAM1 */
  sram2(rwx)   : ORIGIN = 0x30020000, LENGTH = 16k     /* AHB SRAM2 */
  sram4(rwx)   : ORIGIN = 0x38000000, LENGTH = 64k     /* AHB SRAM4 */
  itcm(rwx)    : ORIGIN = 0x00000000, LENGTH = 64k     /* ITCM RAM */
  backup(rwx)  : ORIGIN = 0x38800000, LENGTH = 4k      /* Backup SRAM */
}

/* Use DTCM as primary RAM for stack */
_estack     = ORIGIN(dtcm) + LENGTH(dtcm);    /* stack points to end of DTCM */

SECTIONS {
  .vectors  : { KEEP(*(.vectors)) }   > flash
  .text     : { *(.text*) }           > flash
  .rodata   : { *(.rodata*) }         > flash

  .data : {
    _sdata = .;   /* .data section start */
    *(.first_data)
    *(.data SORT(.data.*))
    _edata = .;  /* .data section end */
  } > dtcm AT > flash
  _sidata = LOADADDR(.data);

  .bss : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > dtcm

  /* Optional sections for other RAM regions */
  .sram1_data (NOLOAD) : {
    *(.sram1_data)
  } > sram1

  .sram2_data (NOLOAD) : {
    *(.sram2_data)
  } > sram2

  .backup_data (NOLOAD) : {
    *(.backup_data)
  } > backup

  . = ALIGN(8);
  _end = .;     /* for cmsis_gcc.h  */
}