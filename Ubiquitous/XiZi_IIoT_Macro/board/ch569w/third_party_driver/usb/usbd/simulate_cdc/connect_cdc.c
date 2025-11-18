/*
 * Copyright (c) 2020 AIIT XUOS Lab
 * XiUOS is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan
 * PSL v2. You may obtain a copy of Mulan PSL v2 at:
 *        http://license.coscl.org.cn/MulanPSL2
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE. See the
 * Mulan PSL v2 for more details.
 */

/*
 * @Note
 *  Example routine to emulate a simulate USB-CDC Device,Use UART2(PA2/PA3).
*/

#include "xizi.h"
#include "board.h"
#include "shell.h"

#include "CH56x_common.h"
#include "CH56x_usb30.h"
#include "cdc.h"


/*******************************************************************************
 * @fn        main
 *
 * @brief     main program
 *
 * @return    None
 */
int UsbdCdcTest()
{
    KPrintf("UsbdCdcTest start\n");
    KPrintf("CH56x USB3.0 & USB2.0 device test(80MHz) !\n");

    /* USB initialization */
    TMR2_TimerInit1();
    R32_USB_CONTROL = 0;
    PFIC_EnableIRQ(USBSS_IRQn);
    PFIC_EnableIRQ(LINK_IRQn);
    PFIC_EnableIRQ(TMR0_IRQn);
    R8_TMR0_INTER_EN = 1;
    TMR0_TimerInit( 67000000 );
    USB30D_init(ENABLE);
    KPrintf("UsbdCdcTest USB30D_init OK\n");

    while(1)
    {
        CDC_Uart_Deal();
    }
}


SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 UsbdCdcTest, UsbdCdcTest, test usbd cdc);
