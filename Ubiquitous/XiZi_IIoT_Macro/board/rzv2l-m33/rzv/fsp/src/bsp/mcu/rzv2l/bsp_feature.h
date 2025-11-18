/***********************************************************************************************************************
 * Copyright [2020-2021] Renesas Electronics Corporation and/or its affiliates.  All Rights Reserved.
 *
 * This software and documentation are supplied by Renesas Electronics Corporation and/or its affiliates and may only
 * be used with products of Renesas Electronics Corp. and its affiliates ("Renesas").  No other uses are authorized.
 * Renesas products are sold pursuant to Renesas terms and conditions of sale.  Purchasers are solely responsible for
 * the selection and use of Renesas products and Renesas assumes no liability.  No license, express or implied, to any
 * intellectual property right is granted by Renesas.  This software is protected under all applicable laws, including
 * copyright laws. Renesas reserves the right to change or discontinue this software and/or this documentation.
 * THE SOFTWARE AND DOCUMENTATION IS DELIVERED TO YOU "AS IS," AND RENESAS MAKES NO REPRESENTATIONS OR WARRANTIES, AND
 * TO THE FULLEST EXTENT PERMISSIBLE UNDER APPLICABLE LAW, DISCLAIMS ALL WARRANTIES, WHETHER EXPLICITLY OR IMPLICITLY,
 * INCLUDING WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT, WITH RESPECT TO THE
 * SOFTWARE OR DOCUMENTATION.  RENESAS SHALL HAVE NO LIABILITY ARISING OUT OF ANY SECURITY VULNERABILITY OR BREACH.
 * TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT WILL RENESAS BE LIABLE TO YOU IN CONNECTION WITH THE SOFTWARE OR
 * DOCUMENTATION (OR ANY PERSON OR ENTITY CLAIMING RIGHTS DERIVED FROM YOU) FOR ANY LOSS, DAMAGES, OR CLAIMS WHATSOEVER,
 * INCLUDING, WITHOUT LIMITATION, ANY DIRECT, CONSEQUENTIAL, SPECIAL, INDIRECT, PUNITIVE, OR INCIDENTAL DAMAGES; ANY
 * LOST PROFITS, OTHER ECONOMIC DAMAGE, PROPERTY DAMAGE, OR PERSONAL INJURY; AND EVEN IF RENESAS HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH LOSS, DAMAGES, CLAIMS OR COSTS.
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * File Name    : bsp_feature.h
 * Version      : 1.00
 * Description  : bsp_feature header
 *********************************************************************************************************************/

#ifndef BSP_FEATURE_H
#define BSP_FEATURE_H

/***********************************************************************************************************************
 * Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private global variables and functions
 **********************************************************************************************************************/
#define BSP_FEATURE_CPG_HAS_ICLK                              (1U)
#define BSP_FEATURE_CPG_HAS_I2CLK                             (1U)
#define BSP_FEATURE_CPG_HAS_GCLK                              (1U)
#define BSP_FEATURE_CPG_HAS_S0CLK                             (1U)
#define BSP_FEATURE_CPG_HAS_SPI0CLK                           (1U)
#define BSP_FEATURE_CPG_HAS_SPI1CLK                           (1U)
#define BSP_FEATURE_CPG_HAS_SD0CLK                            (1U)
#define BSP_FEATURE_CPG_HAS_SD1CLK                            (1U)
#define BSP_FEATURE_CPG_HAS_M0CLK                             (1U)
#define BSP_FEATURE_CPG_HAS_M1CLK                             (1U)
#define BSP_FEATURE_CPG_HAS_M2CLK                             (1U)
#define BSP_FEATURE_CPG_HAS_M3CLK                             (1U)
#define BSP_FEATURE_CPG_HAS_M4CLK                             (1U)
#define BSP_FEATURE_CPG_HAS_HPCLK                             (1U)
#define BSP_FEATURE_CPG_HAS_TSUCLK                            (1U)
#define BSP_FEATURE_CPG_HAS_ZTCLK                             (1U)
#define BSP_FEATURE_CPG_HAS_P0CLK                             (1U)
#define BSP_FEATURE_CPG_HAS_P1CLK                             (1U)
#define BSP_FEATURE_CPG_HAS_P2CLK                             (1U)
#define BSP_FEATURE_CPG_HAS_ATCLK                             (1U)
#define BSP_FEATURE_CPG_HAS_OSCCLK                            (1U)

#define BSP_FEATURE_GPT_VALID_CHANNEL_MASK                    (0xFFU)

#define BSP_FEATURE_GTM_VALID_CHANNEL_MASK                    (0x7)

#define BSP_FEATURE_IIC_VALID_CHANNEL_MASK                    (0x0F)

#define BSP_FEATURE_INTC_IRQ_VALID_CHANNEL_MASK               (0xFFU)

#define BSP_FEATURE_IOPORT_HAS_ETHERNET                       (1U)
#define BSP_FEATURE_IOPORT_HAS_SD                             (1U)
#define BSP_FEATURE_IOPORT_HAS_QSPI                           (1U)

#define BSP_FEATURE_MHU_NS_VALID_CHANNEL_MASK                 (0x3AU)
#define BSP_FEATURE_MHU_NS_SWINT_GET_VALID_CHANNEL_MASK       (0x0FU)
#define BSP_FEATURE_MHU_NS_SWINT_SET_VALID_CHANNEL_MASK       (0x0FU)
#define BSP_FEATURE_MHU_S_VALID_CHANNEL_MASK                  (0x3AU)

#define BSP_FEATURE_POEG_CHANNEL_MASK                         (0xFU)

#define BSP_FEATURE_RSPI_VALID_CHANNELS_MASK                  (0x7)
#define BSP_FEATURE_RSPI_CLOCK                                (FSP_PRIV_CLOCK_P0CLK)
#define BSP_FEATURE_RSPI_HAS_SSL_LEVEL_KEEP                   (1U)
#define BSP_FEATURE_RSPI_SSL_LEVEL_KEEP_VALID_CHANNEL_MASK    (0x7U)

#define BSP_FEATURE_SCIF_CHANNELS                             (0x1FU)
#define BSP_FEATURE_SCIF_CHANNELS_HAS_RTSCTS                  (0x07U)
#define BSP_FEATURE_SCIF_CLOCK                                (FSP_PRIV_CLOCK_P0CLK)

#define BSP_FEATURE_CANFD_NUM_CHANNELS                        (2)

#define BSP_FEATURE_TZ_HAS_TRUSTZONE                          (1U)

#endif                                 /* BSP_FEATURE_H */
