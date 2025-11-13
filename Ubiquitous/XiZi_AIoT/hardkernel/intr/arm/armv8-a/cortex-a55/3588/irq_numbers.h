/**
 * @file irq_numbers.c
 * @brief irq numbers
 * @version 3.0
 * @author AIIT XUOS Lab
 * @date 2023.08.25
 */

/*************************************************
File name: irq_numbers.c
Description: irq numbers
Others:
History:
1. Date: 2023-08-28
Author: AIIT XUOS Lab
Modification:
1. Add HW_NR_IRQS
*************************************************/
#if !defined(__IRQ_NUMBERS_H__)
#define __IRQ_NUMBERS_H__

#define HW_NR_IRQS NR_OK1028_INTRS

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief i.MX6 interrupt numbers.
//!
//! This enumeration lists the numbers for all of the interrupts available on the i.MX6 series.
//! Use these numbers when specifying an interrupt to the GIC.
//!
//! The first 16 interrupts are special in that they are reserved for software interrupts generated
//! by the SWI instruction.

enum _ls_interrupts {
    SW_INTERRUPT_0 = 0, //!< Software interrupt 0.
    SW_INTERRUPT_1 = 1, //!< Software interrupt 1.
    SW_INTERRUPT_2 = 2, //!< Software interrupt 2.
    SW_INTERRUPT_3 = 3, //!< Software interrupt 3.
    SW_INTERRUPT_4 = 4, //!< Software interrupt 4.
    SW_INTERRUPT_5 = 5, //!< Software interrupt 5.
    SW_INTERRUPT_6 = 6, //!< Software interrupt 6.
    SW_INTERRUPT_7 = 7, //!< Software interrupt 7.
    SW_INTERRUPT_8 = 8, //!< Software interrupt 8.
    SW_INTERRUPT_9 = 9, //!< Software interrupt 9.
    SW_INTERRUPT_10 = 10, //!< Software interrupt 10.
    SW_INTERRUPT_11 = 11, //!< Software interrupt 11.
    SW_INTERRUPT_12 = 12, //!< Software interrupt 12.
    SW_INTERRUPT_13 = 13, //!< Software interrupt 13.
    SW_INTERRUPT_14 = 14, //!< Software interrupt 14.
    SW_INTERRUPT_15 = 15, //!< Software interrupt 15.
    RSVD_INTERRUPT_16 = 16, //!< Reserved.
    RSVD_INTERRUPT_17 = 17, //!< Reserved.
    RSVD_INTERRUPT_18 = 18, //!< Reserved.
    RSVD_INTERRUPT_19 = 19, //!< Reserved.
    RSVD_INTERRUPT_20 = 20, //!< Reserved.
    RSVD_INTERRUPT_21 = 21, //!< Reserved.

    LS_INT_DEBUG_CC = 22, //!<(cluster-internal) COMMIRQ - Debug communications channel
    LS_INT_PMU = 23, //!<(cluster-internal) PMUIRQ - Perfmon*
    LS_INT_CTI = 24, //!<(cluster-internal) CTIIRQ - Cross-trigger interface*
    LS_INT_VMI = 25, //!<(cluster-internal) VCPUMNTIRQ -Virtual maintenance interface*

    LS_INT_WDOG = 28, //!< Watchdog timer
    LS_INT_SEC_PHY_TIMER = 29, //!<(cluster-internal) CNTPSIRQ - EL1 Secure physical timer event*
    LS_INT_NON_SEC_PHY_TIMER = 30, //!<(cluster-internal) CNTPNSIRQ - EL1 Non-secure physical timer event*
    RSVD_INTERRUPT_31 = 31, //!< Reserved.
    RSVD_INTERRUPT_32 = 32, //!< Reserved.
    RSVD_INTERRUPT_33 = 33, //!< Reserved.
    RSVD_INTERRUPT_34 = 34, //!< Reserved.
    RSVD_INTERRUPT_35 = 35, //!< Reserved.
    RSVD_INTERRUPT_36 = 36, //!< Reserved.
    RSVD_INTERRUPT_37 = 37, //!< Reserved.
    RSVD_INTERRUPT_38 = 38, //!< Reserved.
    RSVD_INTERRUPT_39 = 39, //!< Reserved.
    RSVD_INTERRUPT_40 = 40, //!< Reserved.
    RSVD_INTERRUPT_41 = 41, //!< Reserved.
    RSVD_INTERRUPT_42 = 42, //!< Reserved.

    LS_INT_DUART1 = 64, // Logical OR of DUART1 interrupt requests.

    LS_INT_I2C1_2 = 66, //!< I2C1 and I2C2 ORed
    LS_INT_I2C3_4 = 67, //!< I2C3 and I2C4 ORed
    LS_INT_GPIO1_2 = 68, //!< GPIO1 and GPIO2 ORed
    LS_INT_GPIO3 = 69, //!< GPIO3

    LS_INT_FLETIMER1 = 76, //!< ORed all Flextimer 1 interrupt signals
    LS_INT_FLETIMER2 = 77, //!< ORed all Flextimer 2 interrupt signals
    LS_INT_FLETIMER3 = 78, //!< ORed all Flextimer 3 interrupt signals
    LS_INT_FLETIMER4 = 79, //!< ORed all Flextimer 4 interrupt signals

    LS_INT_I2C5_6 = 106, //!< I2C5 and I2C6 ORed
    LS_INT_I2C7_8 = 107, //!< I2C7 and I2C8 ORed

    LS_INT_USB3_1 = 112, //!< USB1 ORed INT
    LS_INT_USB3_2 = 113, //!< USB2 ORed INT

    LS_INT_LPUART1 = 264, //!< LPUART1 interrupt request.
    LS_INT_LPUART2 = 265, //!< LPUART1 interrupt request.
    LS_INT_LPUART3 = 266, //!< LPUART1 interrupt request.
    LS_INT_LPUART4 = 267, //!< LPUART1 interrupt request.
    LS_INT_LPUART5 = 268, //!< LPUART1 interrupt request.
    LS_INT_LPUART6 = 269, //!< LPUART1 interrupt request.

    NR_OK1028_INTRS,

};

#endif //__IRQ_NUMBERS_H__