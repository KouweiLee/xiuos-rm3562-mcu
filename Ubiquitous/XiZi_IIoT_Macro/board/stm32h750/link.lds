/*
 * linker script for STM32F10x with GNU ld
 */

/* Program Entry, set to mark it as "used" and avoid gc */
MEMORY
{
    flash (rx) : ORIGIN = 0x08000000, LENGTH = 128k /* 128KB flash */
    sram (rw) : ORIGIN = 0x24000000, LENGTH =  512k /* 512K sram */
}
OUTPUT_ARCH(arm)

ENTRY(Reset_Handler)
_system_stack_size = 0x1000;

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))            /* Startup code */

        . = ALIGN(4);
        *(.text)                        /* remaining code */
        *(.text.*)                      /* remaining code */
        *(.rodata)                      /* read-only data (constants) */
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)

        /* section information for shell */
        . = ALIGN(4);
        _shell_command_start = .;
        KEEP (*(shellCommand))
        _shell_command_end = .;
        . = ALIGN(4);
      
        __isrtbl_idx_start = .;
        KEEP(*(.isrtbl.idx))
        __isrtbl_start = .;
        KEEP(*(.isrtbl))
        __isrtbl_end = .;
        . = ALIGN(4);


        PROVIDE(_etext =  ABSOLUTE(.));

        __etext = .;
    } > flash = 0

    /* .ARM.exidx is sorted, so has to go in its own output section.  */
     __exidx_start = .;
    .ARM.exidx :
    {
        PROVIDE(__exidx_start = ABSOLUTE(.));
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        _sidata = .;
        PROVIDE(__exidx_end = ABSOLUTE(.));
    } > flash

    /* .data section which is used for initialized data */

    .data : AT (__exidx_end )
    {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        __data_start__ = . ;

        *(.data)
        *(.data.*)
        
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        __data_end__ = . ;
    } >sram

    
    .bss :
    {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        __bss_start__ = .;
        _sbss = .;

        *(.bss)
        *(.bss.*)
        *(COMMON)

        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _ebss = . ;
        __bss_end__ = .;
    } > sram
    .stack : 
    {
        . = ALIGN(8);
        /* cpu stack */
        . = . + _system_stack_size;
        __StackTop = .;
    } > sram

    _end = .;

}
