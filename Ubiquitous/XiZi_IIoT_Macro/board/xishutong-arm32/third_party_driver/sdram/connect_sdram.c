/*
* Copyright (c) 2020 AIIT XUOS Lab
* XiUOS is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*        http://license.coscl.org.cn/MulanPSL2
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
* See the Mulan PSL v2 for more details.
*/

/**
* @file connect_sdram.c
* @brief support xishutong-arm32 sdram function, using IS42S16400J
* @version 3.0 
* @author AIIT XUOS Lab
* @date 2023-12-6
*/

#include <connect_sdram.h>

/**
 * @defgroup IS42S16400J_Mode_Register_Field IS42S16400J Mode Register Field
 * @{
 */
/* IS42S16400J burst length definition */
#define IS42S16400J_MR_BURST_1BEAT                  (0UL)
#define IS42S16400J_MR_BURST_2BEAT                  (1UL)
#define IS42S16400J_MR_BURST_4BEAT                  (2UL)
#define IS42S16400J_MR_BURST_8BEAT                  (3UL)
#define IS42S16400J_MR_BURST_LEN_FULLPAGE           (7UL)

/* IS42S16400J burst type definition */
#define IS42S16400J_MR_BURST_TYPE_SEQUENTIAL        (0UL)
#define IS42S16400J_MR_BURST_TYPE_INTERLEAVED       (1UL << 3)

/* IS42S16400J CAS latency definition */
#define IS42S16400J_MR_CAS_LATENCY_2                (2UL << 4)
#define IS42S16400J_MR_CAS_LATENCY_3                (3UL << 4)

/* IS42S16400J write burst mode definition */
#define IS42S16400J_MR_WRITEBURST_PROGRAMMED        (0UL)
#define IS42S16400J_MR_WRITEBURST_SINGLE            (1UL << 9)

/**
 * @defgroup IS42S16400J_Memory_Size IS42S16400J Memory Size
 * @{
 */
#define BSP_IS42S16400J_SIZE                        (8UL * 1024UL * 1024UL)     /* 8MBytes*/

/**
 * @defgroup DMC_Max_Timeout DMC Max Timeout
 * @{
 */
#define DMC_MAX_TIMEOUT                             (0x100000UL)

#ifdef EXTSRAM_TEST
/**
 * @defgroup SDRAM write/read byte test
 * @{
 */
#define DATA_BUF_LEN                                (0x8000UL)
static uint8_t m_au8ReadData[DATA_BUF_LEN];
static uint8_t m_au8WriteData[DATA_BUF_LEN];
static uint32_t m_u32ByteTestErrorCnt = 0UL;
#endif

/**
 * @defgroup DMC_Interface_Pin DMC Interface Pin
 * @{
 */
#define BSP_IS42S16400J_CKE_PORT        (GPIO_PORT_C)   /* PC03 - EXMC_ALE */
#define BSP_IS42S16400J_CKE_PIN         (GPIO_PIN_03)

#define BSP_IS42S16400J_CLK_PORT        (GPIO_PORT_G)   /* PD03 - EXMC_CLK */
#define BSP_IS42S16400J_CLK_PIN         (GPIO_PIN_08)

#define BSP_IS42S16400J_DQM0_PORT       (GPIO_PORT_E)   /* PE00 - EXMC_CE4 */
#define BSP_IS42S16400J_DQM0_PIN        (GPIO_PIN_00)
#define BSP_IS42S16400J_DQM1_PORT       (GPIO_PORT_E)   /* PE01 - EXMC_CE5 */
#define BSP_IS42S16400J_DQM1_PIN        (GPIO_PIN_01)

#define BSP_IS42S16400J_BA0_PORT        (GPIO_PORT_D)   /* PD11 - EXMC_ADD16 */
#define BSP_IS42S16400J_BA0_PIN         (GPIO_PIN_11)
#define BSP_IS42S16400J_BA1_PORT        (GPIO_PORT_D)   /* PD12 - EXMC_ADD17 */
#define BSP_IS42S16400J_BA1_PIN         (GPIO_PIN_12)

#define BSP_IS42S16400J_CS1_PORT        (GPIO_PORT_G)   /* PG09 - EXMC_CE1 */
#define BSP_IS42S16400J_CS1_PIN         (GPIO_PIN_09)

#define BSP_IS42S16400J_RAS_PORT        (GPIO_PORT_F)   /* PF11 - EXMC_OE */
#define BSP_IS42S16400J_RAS_PIN         (GPIO_PIN_11)

#define BSP_IS42S16400J_CAS_PORT        (GPIO_PORT_G)   /* PG15 - EXMC_BAA */
#define BSP_IS42S16400J_CAS_PIN         (GPIO_PIN_15)

#define BSP_IS42S16400J_WE_PORT         (GPIO_PORT_C)   /* PC00 - EXMC_WE */
#define BSP_IS42S16400J_WE_PIN          (GPIO_PIN_00)

#define BSP_IS42S16400J_ADD0_PORT       (GPIO_PORT_F)   /* PF00 - EXMC_ADD0 */
#define BSP_IS42S16400J_ADD0_PIN        (GPIO_PIN_00)
#define BSP_IS42S16400J_ADD1_PORT       (GPIO_PORT_F)   /* PF01 - EXMC_ADD1 */
#define BSP_IS42S16400J_ADD1_PIN        (GPIO_PIN_01)
#define BSP_IS42S16400J_ADD2_PORT       (GPIO_PORT_F)   /* PF02 - EXMC_ADD2 */
#define BSP_IS42S16400J_ADD2_PIN        (GPIO_PIN_02)
#define BSP_IS42S16400J_ADD3_PORT       (GPIO_PORT_F)   /* PF03 - EXMC_ADD3 */
#define BSP_IS42S16400J_ADD3_PIN        (GPIO_PIN_03)
#define BSP_IS42S16400J_ADD4_PORT       (GPIO_PORT_F)   /* PF04 - EXMC_ADD4 */
#define BSP_IS42S16400J_ADD4_PIN        (GPIO_PIN_04)
#define BSP_IS42S16400J_ADD5_PORT       (GPIO_PORT_F)   /* PF05 - EXMC_ADD5 */
#define BSP_IS42S16400J_ADD5_PIN        (GPIO_PIN_05)
#define BSP_IS42S16400J_ADD6_PORT       (GPIO_PORT_F)   /* PF12 - EXMC_ADD6 */
#define BSP_IS42S16400J_ADD6_PIN        (GPIO_PIN_12)
#define BSP_IS42S16400J_ADD7_PORT       (GPIO_PORT_F)   /* PF13 - EXMC_ADD7 */
#define BSP_IS42S16400J_ADD7_PIN        (GPIO_PIN_13)
#define BSP_IS42S16400J_ADD8_PORT       (GPIO_PORT_F)   /* PF14 - EXMC_ADD8 */
#define BSP_IS42S16400J_ADD8_PIN        (GPIO_PIN_14)
#define BSP_IS42S16400J_ADD9_PORT       (GPIO_PORT_F)   /* PF15 - EXMC_ADD9 */
#define BSP_IS42S16400J_ADD9_PIN        (GPIO_PIN_15)
#define BSP_IS42S16400J_ADD10_PORT      (GPIO_PORT_G)   /* PG00 - EXMC_ADD10 */
#define BSP_IS42S16400J_ADD10_PIN       (GPIO_PIN_00)
#define BSP_IS42S16400J_ADD11_PORT      (GPIO_PORT_G)   /* PG01 - EXMC_ADD11 */
#define BSP_IS42S16400J_ADD11_PIN       (GPIO_PIN_01)

#define BSP_IS42S16400J_DATA0_PORT      (GPIO_PORT_D)   /* PD14 - EXMC_DATA0 */
#define BSP_IS42S16400J_DATA0_PIN       (GPIO_PIN_14)
#define BSP_IS42S16400J_DATA1_PORT      (GPIO_PORT_D)   /* PD15 - EXMC_DATA1 */
#define BSP_IS42S16400J_DATA1_PIN       (GPIO_PIN_15)
#define BSP_IS42S16400J_DATA2_PORT      (GPIO_PORT_D)   /* PD00 - EXMC_DATA2 */
#define BSP_IS42S16400J_DATA2_PIN       (GPIO_PIN_00)
#define BSP_IS42S16400J_DATA3_PORT      (GPIO_PORT_D)   /* PD01 - EXMC_DATA3 */
#define BSP_IS42S16400J_DATA3_PIN       (GPIO_PIN_01)
#define BSP_IS42S16400J_DATA4_PORT      (GPIO_PORT_E)   /* PE07 - EXMC_DATA4 */
#define BSP_IS42S16400J_DATA4_PIN       (GPIO_PIN_07)
#define BSP_IS42S16400J_DATA5_PORT      (GPIO_PORT_E)   /* PE08 - EXMC_DATA5 */
#define BSP_IS42S16400J_DATA5_PIN       (GPIO_PIN_08)
#define BSP_IS42S16400J_DATA6_PORT      (GPIO_PORT_E)   /* PE09 - EXMC_DATA6 */
#define BSP_IS42S16400J_DATA6_PIN       (GPIO_PIN_09)
#define BSP_IS42S16400J_DATA7_PORT      (GPIO_PORT_E)   /* PE10 - EXMC_DATA7 */
#define BSP_IS42S16400J_DATA7_PIN       (GPIO_PIN_10)
#define BSP_IS42S16400J_DATA8_PORT      (GPIO_PORT_E)   /* PE11 - EXMC_DATA8 */
#define BSP_IS42S16400J_DATA8_PIN       (GPIO_PIN_11)
#define BSP_IS42S16400J_DATA9_PORT      (GPIO_PORT_E)   /* PE12 - EXMC_DATA9 */
#define BSP_IS42S16400J_DATA9_PIN       (GPIO_PIN_12)
#define BSP_IS42S16400J_DATA10_PORT     (GPIO_PORT_E)   /* PE13 - EXMC_DATA10 */
#define BSP_IS42S16400J_DATA10_PIN      (GPIO_PIN_13)
#define BSP_IS42S16400J_DATA11_PORT     (GPIO_PORT_E)   /* PE14 - EXMC_DATA11 */
#define BSP_IS42S16400J_DATA11_PIN      (GPIO_PIN_14)
#define BSP_IS42S16400J_DATA12_PORT     (GPIO_PORT_E)   /* PE15 - EXMC_DATA12 */
#define BSP_IS42S16400J_DATA12_PIN      (GPIO_PIN_15)
#define BSP_IS42S16400J_DATA13_PORT     (GPIO_PORT_D)   /* PD08 - EXMC_DATA13 */
#define BSP_IS42S16400J_DATA13_PIN      (GPIO_PIN_08)
#define BSP_IS42S16400J_DATA14_PORT     (GPIO_PORT_D)   /* PD09 - EXMC_DATA14 */
#define BSP_IS42S16400J_DATA14_PIN      (GPIO_PIN_09)
#define BSP_IS42S16400J_DATA15_PORT     (GPIO_PORT_D)   /* PD10 - EXMC_DATA15 */
#define BSP_IS42S16400J_DATA15_PIN      (GPIO_PIN_10)

/**
 * @brief  Initialize DMC port.
 * @param  None
 * @retval None
 */
static void BSP_DMC_PortInit(void)
{
    stc_gpio_init_t stcGpioInit;

    /************************* Set pin drive capacity *************************/
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDrv = PIN_HIGH_DRV;

    /* DMC_CKE */
    (void)GPIO_Init(BSP_IS42S16400J_CKE_PORT, BSP_IS42S16400J_CKE_PIN, &stcGpioInit);

    /* DMC_CLK */
    (void)GPIO_Init(BSP_IS42S16400J_CLK_PORT, BSP_IS42S16400J_CLK_PIN, &stcGpioInit);

    /* DMC_LDQM && DMC_UDQM */
    (void)GPIO_Init(BSP_IS42S16400J_DQM0_PORT, BSP_IS42S16400J_DQM0_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DQM1_PORT, BSP_IS42S16400J_DQM1_PIN, &stcGpioInit);

    /* DMC_BA[0:1] */
    (void)GPIO_Init(BSP_IS42S16400J_BA0_PORT, BSP_IS42S16400J_BA0_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_BA1_PORT, BSP_IS42S16400J_BA1_PIN, &stcGpioInit);

    /* DMC_CAS && DMC_RAS */
    (void)GPIO_Init(BSP_IS42S16400J_CAS_PORT, BSP_IS42S16400J_CAS_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_RAS_PORT, BSP_IS42S16400J_RAS_PIN, &stcGpioInit);

    /* DMC_WE */
    (void)GPIO_Init(BSP_IS42S16400J_WE_PORT, BSP_IS42S16400J_WE_PIN, &stcGpioInit);

    /* DMC_DATA[0:15] */
    (void)GPIO_Init(BSP_IS42S16400J_DATA0_PORT, BSP_IS42S16400J_DATA0_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA1_PORT, BSP_IS42S16400J_DATA1_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA2_PORT, BSP_IS42S16400J_DATA2_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA3_PORT, BSP_IS42S16400J_DATA3_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA4_PORT, BSP_IS42S16400J_DATA4_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA5_PORT, BSP_IS42S16400J_DATA5_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA6_PORT, BSP_IS42S16400J_DATA6_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA7_PORT, BSP_IS42S16400J_DATA7_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA8_PORT, BSP_IS42S16400J_DATA8_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA9_PORT, BSP_IS42S16400J_DATA9_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA10_PORT, BSP_IS42S16400J_DATA10_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA11_PORT, BSP_IS42S16400J_DATA11_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA12_PORT, BSP_IS42S16400J_DATA12_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA13_PORT, BSP_IS42S16400J_DATA13_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA14_PORT, BSP_IS42S16400J_DATA14_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_DATA15_PORT, BSP_IS42S16400J_DATA15_PIN, &stcGpioInit);

    /* DMC_ADD[0:11]*/
    (void)GPIO_Init(BSP_IS42S16400J_ADD0_PORT, BSP_IS42S16400J_ADD0_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_ADD1_PORT, BSP_IS42S16400J_ADD1_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_ADD2_PORT, BSP_IS42S16400J_ADD2_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_ADD3_PORT, BSP_IS42S16400J_ADD3_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_ADD4_PORT, BSP_IS42S16400J_ADD4_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_ADD5_PORT, BSP_IS42S16400J_ADD5_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_ADD6_PORT, BSP_IS42S16400J_ADD6_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_ADD7_PORT, BSP_IS42S16400J_ADD7_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_ADD8_PORT, BSP_IS42S16400J_ADD8_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_ADD9_PORT, BSP_IS42S16400J_ADD9_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_ADD10_PORT, BSP_IS42S16400J_ADD10_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_IS42S16400J_ADD11_PORT, BSP_IS42S16400J_ADD11_PIN, &stcGpioInit);

    /************************** Set EXMC pin function *************************/
    /* DMC_CKE */
    GPIO_SetFunc(BSP_IS42S16400J_CKE_PORT, BSP_IS42S16400J_CKE_PIN, GPIO_FUNC_12);

    /* DMC_CLK */
    GPIO_SetFunc(BSP_IS42S16400J_CLK_PORT, BSP_IS42S16400J_CLK_PIN, GPIO_FUNC_12);

    /* DMC_LDQM && DMC_UDQM */
    GPIO_SetFunc(BSP_IS42S16400J_DQM0_PORT, BSP_IS42S16400J_DQM0_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DQM1_PORT, BSP_IS42S16400J_DQM1_PIN, GPIO_FUNC_12);

    /* DMC_BA[0:1] */
    GPIO_SetFunc(BSP_IS42S16400J_BA0_PORT, BSP_IS42S16400J_BA0_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_BA1_PORT, BSP_IS42S16400J_BA1_PIN, GPIO_FUNC_12);

    /* DMC_CS */
    GPIO_SetFunc(BSP_IS42S16400J_CS1_PORT, BSP_IS42S16400J_CS1_PIN, GPIO_FUNC_12);

    /* DMC_CAS && DMC_RAS */
    GPIO_SetFunc(BSP_IS42S16400J_CAS_PORT, BSP_IS42S16400J_CAS_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_RAS_PORT, BSP_IS42S16400J_RAS_PIN, GPIO_FUNC_12);

    /* DMC_WE */
    GPIO_SetFunc(BSP_IS42S16400J_WE_PORT, BSP_IS42S16400J_WE_PIN, GPIO_FUNC_12);

    /* DMC_DATA[0:15] */
    GPIO_SetFunc(BSP_IS42S16400J_DATA0_PORT, BSP_IS42S16400J_DATA0_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA1_PORT, BSP_IS42S16400J_DATA1_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA2_PORT, BSP_IS42S16400J_DATA2_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA3_PORT, BSP_IS42S16400J_DATA3_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA4_PORT, BSP_IS42S16400J_DATA4_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA5_PORT, BSP_IS42S16400J_DATA5_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA6_PORT, BSP_IS42S16400J_DATA6_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA7_PORT, BSP_IS42S16400J_DATA7_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA8_PORT, BSP_IS42S16400J_DATA8_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA9_PORT, BSP_IS42S16400J_DATA9_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA10_PORT, BSP_IS42S16400J_DATA10_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA11_PORT, BSP_IS42S16400J_DATA11_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA12_PORT, BSP_IS42S16400J_DATA12_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA13_PORT, BSP_IS42S16400J_DATA13_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA14_PORT, BSP_IS42S16400J_DATA14_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_DATA15_PORT, BSP_IS42S16400J_DATA15_PIN, GPIO_FUNC_12);

    /* DMC_ADD[0:11]*/
    GPIO_SetFunc(BSP_IS42S16400J_ADD0_PORT, BSP_IS42S16400J_ADD0_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_ADD1_PORT, BSP_IS42S16400J_ADD1_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_ADD2_PORT, BSP_IS42S16400J_ADD2_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_ADD3_PORT, BSP_IS42S16400J_ADD3_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_ADD4_PORT, BSP_IS42S16400J_ADD4_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_ADD5_PORT, BSP_IS42S16400J_ADD5_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_ADD6_PORT, BSP_IS42S16400J_ADD6_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_ADD7_PORT, BSP_IS42S16400J_ADD7_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_ADD8_PORT, BSP_IS42S16400J_ADD8_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_ADD9_PORT, BSP_IS42S16400J_ADD9_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_ADD10_PORT, BSP_IS42S16400J_ADD10_PIN, GPIO_FUNC_12);
    GPIO_SetFunc(BSP_IS42S16400J_ADD11_PORT, BSP_IS42S16400J_ADD11_PIN, GPIO_FUNC_12);
}

/**
 * @brief  SDRAM IS42S16400J initialization sequence.
 * @param  [in] u32Chip                     The command chip number.
 *         This parameter can be one of the following values:
 *           @arg EXMC_DMC_CHIP0:           Chip 0
 *           @arg EXMC_DMC_CHIP1:           Chip 1
 *           @arg EXMC_DMC_CHIP2:           Chip 2
 *           @arg EXMC_DMC_CHIP3:           Chip 3
 * @param  [in] u32Bank                     The command bank.
 *         This parameter can be one of the following values:
 *           @arg EXMC_DMC_BANK0:           Bank 0
 *           @arg EXMC_DMC_BANK1:           Bank 1
 *           @arg EXMC_DMC_BANK2:           Bank 2
 *           @arg EXMC_DMC_BANK3:           Bank 3
 * @param  [in] u32MdRegValue               The SDRAM mode register value
 * @retval None
 */
static void BSP_SDRAM_InitSequence(uint32_t u32Chip, uint32_t u32Bank, uint32_t u32MdRegValue)
{
    /* SDRAM initialization sequence:
       CMD NOP->PrechargeAll->AutoRefresh->AutoRefresh->MdRegConfig->NOP */
    (void)EXMC_DMC_SetCommand(u32Chip, u32Bank, EXMC_DMC_CMD_NOP, 0UL);
    (void)EXMC_DMC_SetCommand(u32Chip, u32Bank, EXMC_DMC_CMD_PRECHARGE_ALL, 0UL);
    (void)EXMC_DMC_SetCommand(u32Chip, u32Bank, EXMC_DMC_CMD_AUTO_REFRESH, 0UL);
    (void)EXMC_DMC_SetCommand(u32Chip, u32Bank, EXMC_DMC_CMD_AUTO_REFRESH, 0UL);
    (void)EXMC_DMC_SetCommand(u32Chip, u32Bank, EXMC_DMC_CMD_MDREG_CONFIG, u32MdRegValue);
    (void)EXMC_DMC_SetCommand(u32Chip, u32Bank, EXMC_DMC_CMD_NOP, 0UL);
}

/**
 * @brief  Get memory information.
 * @param  [out] pu32MemoryStartAddr    The pointer for memory start address
 * @param  [out] pu32MemoryByteSize     The pointer for memory size(unit: Byte)
 * @retval None
 */
void BSP_IS42S16400J_GetMemInfo(uint32_t *pu32MemoryStartAddr, uint32_t *pu32MemoryByteSize)
{
    if (NULL != pu32MemoryStartAddr) {
        *pu32MemoryStartAddr = EXMC_DMC_GetChipStartAddr(EXMC_DMC_CHIP1);
    }

    if (NULL != pu32MemoryByteSize) {
        *pu32MemoryByteSize = BSP_IS42S16400J_SIZE - 1UL;
    }
}

#ifdef EXTSRAM_TEST
/**
 * @brief  Write memory for byte.
 * @param  [in] u32Addr                 Memory address to write
 * @param  [in] au8Data                 Pointer to source buffer
 * @param  [in] u32Len                  Length of the buffer to write to memory
 * @retval int32_t:
 *           - LL_OK:                   Write successfully.
 *           - LL_ERR_INVD_PARAM:       If one of following cases matches:
 *                                      -The pointer au8Data value is NULL.
 *                                      -The variable u32Len value is 0.
 */
static int32_t MEMORY_Write8(uint32_t u32Addr, const uint8_t au8Data[], uint32_t u32Len)
{
    uint32_t i;
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    if ((au8Data != NULL) && (0UL != u32Len)) {
        for (i = 0UL; i < u32Len; i++) {
            *(uint8_t *)(u32Addr + i) = au8Data[i];
        }
        i32Ret = LL_OK;
    }

    return i32Ret;
}

/**
 * @brief  Read memory for byte.
 * @param  [in] u32Addr                 Memory address to read
 * @param  [out] au8Data                Pointer to buffer
 * @param  [in] u32Len                  Length of the buffer to read from memory
 * @retval int32_t:
 *           - LL_OK:                   Read successfully.
 *           - LL_ERR_INVD_PARAM:       If one of following cases matches:
 *                                      -The pointer au8Data value is NULL.
 *                                      -The variable u32Len value is 0.
 */
static int32_t MEMORY_Read8(uint32_t u32Addr, uint8_t au8Data[], uint32_t u32Len)
{
    uint32_t i;
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    if ((NULL != au8Data) && (0UL != u32Len)) {
        for (i = 0UL; i < u32Len; i++) {
            au8Data[i] = *(uint8_t *)(u32Addr + i);
        }
        i32Ret = LL_OK;
    }

    return i32Ret;
}

/**
 * @brief  Access memory for byte.
 * @param  [in] u3AccessAddr:           Memory address
 * @param  [in] NumBytes:               Access size(unit: byte)
 * @retval count for reading and writing dada error
 */
static uint32_t MEMORY_Access8(uint32_t u3AccessAddr, uint32_t NumBytes)
{
    uint32_t i;
    uint32_t j;
    uint32_t u32TestErrCnt = 0UL;
    uint32_t u32MemoryAddr = u3AccessAddr;

    /* Initialize test data. */
    for (i = 0UL; i < DATA_BUF_LEN; i++) {
        m_au8ReadData[i] = 0U;
        m_au8WriteData[i] = 0x12U;
    }

    for (i = 0UL; i < NumBytes; i += DATA_BUF_LEN) {
        (void)MEMORY_Write8(u32MemoryAddr, m_au8WriteData, DATA_BUF_LEN);
        (void)MEMORY_Read8(u32MemoryAddr, m_au8ReadData, DATA_BUF_LEN);

        /* Verify data. */
        for (j = 0UL; j < DATA_BUF_LEN; j++) {
            if (m_au8WriteData[j] != m_au8ReadData[j]) {
                u32TestErrCnt++;
                KPrintf("Byte read/write error: address = 0x%.8x; write data = 0x%x; read data = 0x%x\r\n",
                           (unsigned int)(u32MemoryAddr + j), (unsigned int)m_au8WriteData[j], (unsigned int)m_au8ReadData[j]);
            }
        }

        u32MemoryAddr += (DATA_BUF_LEN * sizeof(m_au8ReadData[0]));
        (void)memset(m_au8ReadData, 0, (DATA_BUF_LEN * sizeof(m_au8ReadData[0])));
    }

    return u32TestErrCnt;
}
#endif

int ExtSdramInit(void)
{
    __IO uint32_t u32To = 0UL;
    uint32_t u32MdRegValue;
    int32_t i32Ret = LL_OK;
    stc_exmc_dmc_init_t stcDmcInit;
    stc_exmc_dmc_chip_config_t stcCsConfig;

    uint32_t m_u32StartAddr = 0UL;
    uint32_t m_u32EndAddr = 0UL;
    uint32_t m_u32ByteSize = 0UL;

    /* EXCLK 60MHz */
    CLK_SetClockDiv(CLK_BUS_EXCLK, CLK_EXCLK_DIV4);

    /* Initialization DMC port.*/
    BSP_DMC_PortInit();

    /* Enable DMC clock */
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_DMC, ENABLE);

    /* Enable DMC. */
    EXMC_DMC_Cmd(ENABLE);

    /* Configure DMC width && refresh period & chip & timing. */
    (void)EXMC_DMC_StructInit(&stcDmcInit);
    stcDmcInit.u32RefreshPeriod = 900UL;
    stcDmcInit.u32ColumnBitsNumber = EXMC_DMC_COLUMN_BITS_NUM8;
    stcDmcInit.u32RowBitsNumber = EXMC_DMC_ROW_BITS_NUM12;
    stcDmcInit.u32MemBurst = EXMC_DMC_BURST_1BEAT;
    stcDmcInit.u32AutoRefreshChips = EXMC_DMC_AUTO_REFRESH_2CHIPS;
    stcDmcInit.stcTimingConfig.u8CASL = 2U;
    stcDmcInit.stcTimingConfig.u8DQSS = 0U;
    stcDmcInit.stcTimingConfig.u8MRD = 2U;
    stcDmcInit.stcTimingConfig.u8RAS = 3U;
    stcDmcInit.stcTimingConfig.u8RC = 4U;
    stcDmcInit.stcTimingConfig.u8RCD_B = 1U;
    stcDmcInit.stcTimingConfig.u8RCD_P = 0U;
    stcDmcInit.stcTimingConfig.u8RFC_B = 4U;
    stcDmcInit.stcTimingConfig.u8RFC_P = 0U;
    stcDmcInit.stcTimingConfig.u8RP_B = 1U;
    stcDmcInit.stcTimingConfig.u8RP_P = 0U;
    stcDmcInit.stcTimingConfig.u8RRD = 1U;
    stcDmcInit.stcTimingConfig.u8WR = 2U;
    stcDmcInit.stcTimingConfig.u8WTR = 1U;
    stcDmcInit.stcTimingConfig.u8XP = 1U;
    stcDmcInit.stcTimingConfig.u8XSR = 5U;
    stcDmcInit.stcTimingConfig.u8ESR = 5U;
    (void)EXMC_DMC_Init(&stcDmcInit);

    /* Configure DMC address space. */
    stcCsConfig.u32AddrMask = EXMC_DMC_ADDR_MASK_16MB;
    stcCsConfig.u32AddrMatch = 0x80UL;
    stcCsConfig.u32AddrDecodeMode = EXMC_DMC_CS_DECODE_ROWBANKCOL;
    (void)EXMC_DMC_ChipConfig(EXMC_DMC_CHIP1, &stcCsConfig);

    /* SDRAM initialization sequence. */
    u32MdRegValue = (IS42S16400J_MR_BURST_TYPE_SEQUENTIAL | IS42S16400J_MR_WRITEBURST_PROGRAMMED);
    if (2U == stcDmcInit.stcTimingConfig.u8CASL) {
        u32MdRegValue |= IS42S16400J_MR_CAS_LATENCY_2;
    } else {
        u32MdRegValue |= IS42S16400J_MR_CAS_LATENCY_3;
    }

    if (EXMC_DMC_BURST_1BEAT == stcDmcInit.u32MemBurst) {
        u32MdRegValue |= IS42S16400J_MR_BURST_1BEAT;
    } else if (EXMC_DMC_BURST_2BEAT == stcDmcInit.u32MemBurst) {
        u32MdRegValue |= IS42S16400J_MR_BURST_2BEAT;
    } else if (EXMC_DMC_BURST_4BEAT == stcDmcInit.u32MemBurst) {
        u32MdRegValue |= IS42S16400J_MR_BURST_4BEAT;
    } else {
        u32MdRegValue |= IS42S16400J_MR_BURST_8BEAT;
    }

    BSP_SDRAM_InitSequence(EXMC_DMC_CHIP1, EXMC_DMC_BANK0, u32MdRegValue);

    /* Switch state from configure to ready */
    EXMC_DMC_SetState(EXMC_DMC_CTRL_STATE_GO);
    EXMC_DMC_SetState(EXMC_DMC_CTRL_STATE_WAKEUP);
    EXMC_DMC_SetState(EXMC_DMC_CTRL_STATE_GO);

    /* Check status */
    while (EXMC_DMC_CURR_STATUS_RDY != EXMC_DMC_GetStatus()) {
        if (u32To > DMC_MAX_TIMEOUT) {
            i32Ret = LL_ERR;
            break;
        }
        u32To++;
    }

    BSP_IS42S16400J_GetMemInfo(&m_u32StartAddr, &m_u32ByteSize);
    m_u32EndAddr = (unsigned int)(m_u32StartAddr + m_u32ByteSize - 1UL);

    KPrintf("SDRAM Memory start address: 0x%.8x \r\n", (unsigned int)m_u32StartAddr);
    KPrintf("SDRAM Memory end   address: 0x%.8x \r\n", (unsigned int)m_u32EndAddr);
    KPrintf("SDRAM Memory size  (Bytes): 0x%.8x \r\n", (unsigned int)m_u32ByteSize);

#ifdef EXTSRAM_TEST
    /****************** Access data width: Byte *****************/
    KPrintf("******** Write/read test ********\r\n");
    m_u32ByteTestErrorCnt = MEMORY_Access8(m_u32StartAddr, m_u32ByteSize);
    KPrintf("Byte read/write error data count: %u \r\n", (unsigned int)m_u32ByteTestErrorCnt);
    if (m_u32ByteTestErrorCnt > 0) {
        return LL_ERR;
    }
#endif

    extern void ExtSramInitBoardMemory(void *start_phy_address, void *end_phy_address, uint8 extsram_idx);
    ExtSramInitBoardMemory((void*)(m_u32StartAddr), (void*)(m_u32EndAddr), 0);

    return i32Ret;
}
