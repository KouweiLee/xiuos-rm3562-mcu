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
* @file connect_dac.c
* @brief support to register DAC pointer and function
* @version 3.0
* @author AIIT XUOS Lab
* @date 2023-12-29
*/

#include <connect_dac.h>

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define DAC_UNIT1_PORT                               (GPIO_PORT_A)
#define DAC_UNIT1_CHN1_PIN                           (GPIO_PIN_04)

#define VREFH                                        (3.3F)
#define DAC_CHN1                                     (0U)
#define DAC_CHN2                                     (1U)
#define DAC_DATA_ALIGN_12b_R                         (0U)
#define DAC_DATA_ALIGN_12b_L                         (1U)

#define SUPPORT_AMP
#define SUPPORT_ADP
#define SINGLE_WAVE_DAC_CHN                          (DAC_CHN1)
#define DAC_DATA_ALIGN                               (DAC_DATA_ALIGN_12b_L)

#define SINE_DOT_NUMBER                              (4096U)
#define SINE_NEGATIVE_TO_POSITVE                     (1.0F)

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
typedef enum {
    DAC_Unit1,
    DAC_Unit2,
    DAC_Unit_Max,
}en_dac_unit_t;

typedef enum {
    E_Dac_Single,
    E_Dac_Dual,
}en_dac_cvt_t;

typedef struct {
    CM_DAC_TypeDef *pUnit;
    en_dac_cvt_t enCvtType;
    uint16_t u16Ch;
} stc_dac_handle_t;

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static stc_dac_handle_t m_stcDACHandle[DAC_Unit_Max] = {0};
static uint32_t gu32SinTable[SINE_DOT_NUMBER];
static stc_dac_handle_t *pSingleDac;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @brief   MAU Initialization
 * @param   None
 * @retval  None
 */
static void MauInit(void)
{
    /* Enable MAU peripheral clock. */
    FCG_Fcg0PeriphClockCmd(PWC_FCG0_MAU, ENABLE);
}

/**
 * @brief   MAU De-Initialization
 * @param   None
 * @retval  None
 */
static void MauDeinit(void)
{
    /* Enable MAU peripheral clock. */
    FCG_Fcg0PeriphClockCmd(PWC_FCG0_MAU, DISABLE);
}

/**
 * @brief    Sin table Initialization
 * @param    [in] pSinTable  sin table
 * @param    [in] u32count   number of pSinTable items
 * @retval   None
 */
static void SinTableInit(uint32_t pSinTable[], uint32_t u32count)
{
    uint32_t i;
    uint32_t u32AngAvg = (uint32_t)(float32_t)((float32_t)((float32_t)MAU_SIN_ANGIDX_TOTAL / (float32_t)u32count) + 0.5);
    float32_t fSin;
    for (i = 0U; i < u32count; i++) {
        fSin = (((float32_t)MAU_Sin(CM_MAU, (uint16_t)(u32AngAvg * i))
                 / (float32_t)MAU_SIN_Q15_SCALAR + SINE_NEGATIVE_TO_POSITVE) / VREFH) *
               (float32_t)DAC_DATAREG_VALUE_MAX + 0.5F;

#if (DAC_DATA_ALIGN == DAC_DATA_ALIGN_12b_L)
        {
            pSinTable[i] = (uint32_t)fSin << 4;
        }
#else
        {
            pSinTable[i] = (uint32_t)fSin;
        }
#endif
    }
}

/**
 * @brief    Enable DAC peripheral clock
 * @param    [in] enUnit  The selected DAC unit
 * @retval   None
 */
static void DacPClkEnable(en_dac_unit_t enUnit)
{
    uint32_t u32PClk;
    switch (enUnit) {
        case DAC_Unit1:
            u32PClk = PWC_FCG3_DAC1;
            break;
        case DAC_Unit2:
            u32PClk = PWC_FCG3_DAC2;
            break;
        default:
            u32PClk = PWC_FCG3_DAC1 | PWC_FCG3_DAC2;
            break;
    }
    /* Enable DAC peripheral clock. */
    FCG_Fcg3PeriphClockCmd(u32PClk, ENABLE);
}

/**
 * @brief    Init DAC single channel
 * @param    [in] enUnit  The selected DAC unit
 * @retval   A pointer of DAC handler
 */
static stc_dac_handle_t *DacSingleConversionInit(en_dac_unit_t enUnit)
{
    uint8_t u8Port;
    uint16_t u16Pin;
    stc_dac_handle_t *pDac;

    if (enUnit == DAC_Unit1) {
        pDac = &m_stcDACHandle[DAC_Unit1];
        pDac->pUnit = CM_DAC1;
    } else {
        pDac = &m_stcDACHandle[DAC_Unit2];
        pDac->pUnit = CM_DAC2;
    }
    DacPClkEnable(enUnit);

    pDac->enCvtType = E_Dac_Single;
#if (SINGLE_WAVE_DAC_CHN == DAC_CHN1)
    pDac->u16Ch = DAC_CH1;
#else
    pDac->u16Ch = DAC_CH2;
#endif

    /* Init DAC by default value: source from data register and output enabled*/
    DAC_DeInit(pDac->pUnit);
    stc_dac_init_t stInit;
    (void)DAC_StructInit(&stInit);
    (void)DAC_Init(pDac->pUnit, pDac->u16Ch, &stInit);
#if (DAC_DATA_ALIGN == DAC_DATA_ALIGN_12b_L)
    DAC_DataRegAlignConfig(pDac->pUnit, DAC_DATA_ALIGN_L);
#else
    DAC_DataRegAlignConfig(pDac->pUnit, DAC_DATA_ALIGN_R);
#endif

    /* Set DAC pin attribute to analog */
    if (enUnit == DAC_Unit1) {
        u8Port = DAC_UNIT1_PORT;
#if (SINGLE_WAVE_DAC_CHN == DAC_CHN1)
        u16Pin = DAC_UNIT1_CHN1_PIN;
#endif
    }
    stc_gpio_init_t stcGpioInit;
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(u8Port, u16Pin, &stcGpioInit);

#ifdef SUPPORT_ADP
    /* Set ADC first */
    /* Enable ADC peripheral clock. */
    FCG_Fcg3PeriphClockCmd(PWC_FCG3_ADC1 | PWC_FCG3_ADC2 | PWC_FCG3_ADC3, ENABLE);
    if (CM_ADC1->STR == 0U) {
        if (CM_ADC2->STR == 0U) {
            if (CM_ADC3->STR == 0U) {
                DAC_ADCPrioConfig(pDac->pUnit, DAC_ADP_SELECT_ALL, ENABLE);
                DAC_ADCPrioCmd(pDac->pUnit, ENABLE);
            }
        }
    }
#endif
    return pDac;
}

/**
 * @brief    Start single DAC conversions
 * @param    [in] pDac       A pointer of DAC handler
 * @retval   None
 */
static void DacStartSingleConversion(const stc_dac_handle_t *pDac)
{
    /* Enalbe AMP */
#ifdef SUPPORT_AMP
    (void)DAC_AMPCmd(pDac->pUnit, pDac->u16Ch, ENABLE);
#endif

    (void)DAC_Start(pDac->pUnit, pDac->u16Ch);

#ifdef SUPPORT_AMP
    /* delay 3us before setting data*/
    DDL_DelayMS(1U);
#endif
}

/**
 * @brief    Convert data by single DAC channel
 * @param    [in] pDac       A pointer of DAC handler
 * @param    [in] pDataTable The data table to be converted
 * @param    [in] u32count   Number of data table items
 * @retval   None
 */
__STATIC_INLINE void DacSetSingleConversionData(const stc_dac_handle_t *pDac, uint32_t const pDataTable[], uint32_t u32count)
{
    uint32_t i = 0U;

    for (i = 0U; i < u32count; i++) {
#ifdef SUPPORT_ADP
        uint32_t u32TryCount = 100U;
        while (u32TryCount != 0U) {
            u32TryCount--;
            if (SET != DAC_GetChConvertState(pDac->pUnit, pDac->u16Ch)) {
                break;
            }
        }
#endif
        DAC_SetChData(pDac->pUnit, pDac->u16Ch, (uint16_t)pDataTable[i]);
    }
}

/**
 * @brief    stop DAC conversion
 * @param    [in] pDac A pointer of DAC handler
 * @retval   None
 */
static void DAC_StopConversion(const stc_dac_handle_t *pDac)
{
    if (NULL == pDac) {
        DAC_DeInit(CM_DAC1);
        DAC_DeInit(CM_DAC2);
    } else if (pDac->enCvtType != E_Dac_Dual) {
        (void)DAC_Stop(pDac->pUnit, pDac->u16Ch);
    } else {
        DAC_StopDualCh(pDac->pUnit);
    }
}

static uint32 DacOpen(void *dev)
{
    struct DacHardwareDevice *dac_dev = (struct DacHardwareDevice *)dev;

    /* Init MAU for generating sine data*/
    MauInit();
    /* Init sine data table */
    SinTableInit(gu32SinTable, SINE_DOT_NUMBER);

    /* Init single DAC */
    pSingleDac = DacSingleConversionInit(DAC_Unit1);

    return EOK;
}

static uint32 DacClose(void *dev)
{
    struct DacHardwareDevice *dac_dev = (struct DacHardwareDevice *)dev;
    CM_DAC_TypeDef *DACx  = (CM_DAC_TypeDef *)dac_dev->private_data;
    
    DAC_StopConversion(pSingleDac);
    
    DAC_DeInit(DACx);

    MauDeinit();

    memset(gu32SinTable, 0 , sizeof(gu32SinTable));
    
    return EOK;
}

static uint32 DacWrite(void *dev, struct BusBlockWriteParam *write_param)
{
    struct DacHardwareDevice *dac_dev = (struct DacHardwareDevice *)dev;
    struct HwDac *dac_cfg = (struct HwDac *)dac_dev->haldev.private_data;

    for (int i = 0; i < dac_cfg->digital_data; i ++) {
        DacStartSingleConversion(pSingleDac);
        DacSetSingleConversionData(pSingleDac, &gu32SinTable[i], 1U);
        if (i > SINE_DOT_NUMBER) {
            i = 0;
        }
    }

    return EOK;
}

static uint32 DacDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;

    struct DacDriver *dac_drv = (struct DacDriver *)drv;
    struct DacHardwareDevice *dac_dev = (struct DacHardwareDevice *)dac_drv->driver.owner_bus->owner_haldev;
    struct HwDac *dac_cfg = (struct HwDac *)dac_dev->haldev.private_data;

    switch (configure_info->configure_cmd)
    {
        case OPE_CFG:
            dac_cfg->digital_data = *(uint16 *)configure_info->private_data;
            break;
        default:
            break;
    }

    return ret;
}

static const struct DacDevDone dev_done =
{
    DacOpen,
    DacClose,
    DacWrite,
    NONE,
};

int HwDacInit(void)
{
    x_err_t ret = EOK;

#ifdef BSP_USING_DAC
    static struct DacBus dac_bus;
    static struct DacDriver dac_drv;
    static struct DacHardwareDevice dac_dev;
    static struct HwDac dac_cfg;

    dac_drv.configure = DacDrvConfigure;

    ret = DacBusInit(&dac_bus, DAC_BUS_NAME);
    if (ret != EOK) {
        KPrintf("DAC bus init error %d\n", ret);
        return ERROR;
    }

    ret = DacDriverInit(&dac_drv, DAC_DRIVER_NAME);
    if (ret != EOK) {
        KPrintf("DAC driver init error %d\n", ret);
        return ERROR;
    }
    ret = DacDriverAttachToBus(DAC_DRIVER_NAME, DAC_BUS_NAME);
    if (ret != EOK) {
        KPrintf("DAC driver attach error %d\n", ret);
        return ERROR;
    }

    dac_dev.dac_dev_done = &dev_done;
    dac_cfg.DACx = CM_DAC1;
    dac_cfg.digital_data = 0;

    ret = DacDeviceRegister(&dac_dev, (void *)&dac_cfg, DAC_DEVICE_NAME);
    if (ret != EOK) {
        KPrintf("DAC device register error %d\n", ret);
        return ERROR;
    }
    ret = DacDeviceAttachToBus(DAC_DEVICE_NAME, DAC_BUS_NAME);
    if (ret != EOK) {
        KPrintf("DAC device register error %d\n", ret);
        return ERROR;
    }
#endif 

    return ret;
}
