/*!
 * @file cortexA55.s
 * @brief This file contains cortexA55 functions
 *
 */
/*************************************************
File name: cortexA55.S
Description: This file contains cortexA9 functions
Others: 
History:
1. Date: 2024-05-08
Author: AIIT XUOS Lab
Modification:
1. No modifications
*************************************************/
    .section ".text","ax"

    .global cpu_get_current
  # int cpu_get_current(void)@
  # get current CPU ID
    .func cpu_get_current
cpu_get_current:
    mrs   x0, mpidr_el1
    and   x0, x0, #0xFFF
    lsr   x0, x0, #8
    ret
    .endfunc

    .global psci_call
psci_call:
        hvc #0
        ret


# ------------------------------------------------------------
# End of cortexA55.s
# ------------------------------------------------------------
    .end