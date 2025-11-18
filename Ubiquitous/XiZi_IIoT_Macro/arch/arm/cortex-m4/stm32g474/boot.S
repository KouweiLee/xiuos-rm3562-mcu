.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.word  _sidata
.word  _sdata
.word  _edata
.word  __bss_start
.word  __bss_end

.section .text.Reset_Handler
.weak Reset_Handler
.type Reset_Handler, %function

Reset_Handler:
    ldr   sp, =__stack_tp       
    movs  r1, #0

DataInit:
    ldr   r0, =_sdata
    ldr   r3, =_edata
    adds  r2, r0, r1
    cmp   r2, r3
    bcs   DataInitEnd
    ldr   r3, =_sidata
    ldr   r3, [r3, r1]
    str   r3, [r0, r1]
    adds  r1, r1, #4
    b     DataInit

DataInitEnd:
    ldr   r2, =__bss_start

BSSInit:
    ldr   r3, =__bss_end
    cmp   r2, r3
    bcs   BSSInitEnd
    movs  r3, #0
    str   r3, [r2], #4
    b     BSSInit

BSSInitEnd:
    bl    SystemInit        
    bl    entry             
    bx    lr
    
.size Reset_Handler, .-Reset_Handler