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
* @file connect_adc.c
* @brief support to register ADC pointer and function
* @version 3.0
* @author AIIT XUOS Lab
* @date 2023-12-29
*/

#include <connect_adc.h>

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/* The clock source of ADC. */
#define ADC_CLK_SYS_CLK                 (1U)
#define ADC_CLK_PLLH                    (2U)
#define ADC_CLK_PLLA                    (3U)

/*
 * Selects a clock source according to the application requirements.
 * PCLK4 is the clock for digital interface.
 * PCLK2 is the clock for analog circuit.
 * PCLK4 and PCLK2 are synchronous when the clock source is PLL.
 * PCLK4 : PCLK2 = 1:1, 2:1, 4:1, 8:1, 1:2, 1:4.
 * PCLK2 is in range [1MHz, 60MHz].
 * If the system clock is selected as the ADC clock, macro 'ADC_ADC_CLK' can only be defined as 'CLK_PERIPHCLK_PCLK'.
 * If PLLH is selected as the ADC clock, macro 'ADC_ADC_CLK' can be defined as 'CLK_PERIPHCLK_PLLx'(x=Q, R).
 * If PLLA is selected as the ADC clock, macro 'ADC_ADC_CLK' can be defined as 'CLK_PERIPHCLK_PLLXx'(x=P, Q, R).
 */
#define ADC_CLK_SEL                     (ADC_CLK_SYS_CLK)

#if (ADC_CLK_SEL == ADC_CLK_SYS_CLK)
#define ADC_CLK                         (CLK_PERIPHCLK_PCLK)

#elif (ADC_CLK_SEL == ADC_CLK_PLLH)
#define ADC_CLK                         (CLK_PERIPHCLK_PLLQ)

#elif (ADC_CLK_SEL == ADC_CLK_PLLA)
#define ADC_CLK                         (CLK_PERIPHCLK_PLLXP)

#else
#error "The clock source your selected does not exist!!!"
#endif

/* ADC unit instance for this example. */
#define ADC_UNIT                        (CM_ADC1)
#define ADC_PERIPH_CLK                  (FCG3_PERIPH_ADC1)

/* Selects ADC channels that needed. */
#define ADC_CH_POTENTIOMETER            (ADC_CH3)
#define ADC_CH                          (ADC_CH_POTENTIOMETER)
#define ADC_CH_PORT                     (GPIO_PORT_A)
#define ADC_CH_PIN                      (GPIO_PIN_03)

/* ADC sequence to be used. */
#define ADC_SEQ                         (ADC_SEQ_A)
/* Flag of conversion end. */
#define ADC_EOC_FLAG                    (ADC_FLAG_EOCA)

/* ADC reference voltage. The voltage of pin VREFH. */
#define ADC_VREF                        (3.3F)

/* ADC accuracy(according to the resolution of ADC). */
#define ADC_ACCURACY                    (1UL << 12U)

/* Calculate the voltage(mV). */
#define ADC_CAL_VOL(adcVal)             (uint16_t)((((float32_t)(adcVal) * ADC_VREF) / ((float32_t)ADC_ACCURACY)) * 1000.F)

/* Timeout value. */
#define ADC_TIMEOUT_VAL                 (1000U)

/**
 * @brief  Set specified ADC pin to analog mode.
 * @param  None
 * @retval None
 */
static void AdcSetPinAnalogMode(void)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(ADC_CH_PORT, ADC_CH_PIN, &stcGpioInit);
}

/**
 * @brief  Configures ADC clock.
 * @param  None
 * @retval None
 */
static void AdcClockConfig(void)
{
#if (ADC_CLK_SEL == ADC_CLK_SYS_CLK)
    /*
     * 1. Configures the clock divider of PCLK2 and PCLK4 here or in the function of configuring the system clock.
     *    In this example, the system clock is MRC@8MHz.
     *    PCLK4 is the digital interface clock, and PCLK2 is the analog circuit clock.
     *    Make sure that PCLK2 and PCLK4 meet the following conditions:
     *      PCLK4 : PCLK2 = 1:1, 2:1, 4:1, 8:1, 1:2, 1:4.
     *      PCLK2 is in range [1MHz, 60MHz].
     */
    CLK_SetClockDiv((CLK_BUS_PCLK2 | CLK_BUS_PCLK4), (CLK_PCLK2_DIV8 | CLK_PCLK4_DIV2));

#elif (ADC_CLK_SEL == ADC_CLK_PLLH)
    /*
     * 1. Configures PLLH and the divider of PLLHx(x=Q, R).
     *    PLLHx(x=Q, R) is used as both the digital interface clock and the analog circuit clock.
     *    PLLHx(x=Q, R) must be in range [1MHz, 60MHz] for ADC use.
     *    The input source of PLLH is XTAL(8MHz).
     */
    stc_clock_pll_init_t stcPLLHInit;
    stc_clock_xtal_init_t stcXtalInit;

    /* Configures XTAL. PLLH input source is XTAL. */
    (void)CLK_XtalStructInit(&stcXtalInit);
    stcXtalInit.u8State      = CLK_XTAL_ON;
    stcXtalInit.u8Drv        = CLK_XTAL_DRV_ULOW;
    stcXtalInit.u8Mode       = CLK_XTAL_MD_OSC;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_499US;
    (void)CLK_XtalInit(&stcXtalInit);

    (void)CLK_PLLStructInit(&stcPLLHInit);
    /*
     * PLLHx(x=Q, R) = ((PLL_source / PLLM) * PLLN) / PLLx
     * PLLHQ = (8 / 1) * 80 /16 = 40MHz
     * PLLHR = (8 / 1) * 80 /16 = 40MHz
     */
    stcPLLHInit.u8PLLState = CLK_PLL_ON;
    stcPLLHInit.PLLCFGR = 0UL;
    stcPLLHInit.PLLCFGR_f.PLLM = (1UL  - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLN = (80UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLP = (4UL  - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLQ = (16UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLR = (16UL - 1UL);
    /* stcPLLHInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL; */
    (void)CLK_PLLInit(&stcPLLHInit);

#elif (ADC_CLK_SEL == ADC_CLK_PLLA)
    /*
     * 1. Configures PLLA and the divider of PLLAx(x=P, Q, R).
     *    PLLAx(x=P, Q, R) is used as both the digital interface clock and the analog circuit clock.
     *    PLLAx(x=P, Q, R) must be in range [1MHz, 60MHz] for ADC use.
     *    The input source of PLLA is HRC(16MHz).
     */
    stc_clock_pllx_init_t stcPLLAInit;

    /* Enable HRC(16MHz) for PLLA. */
    CLK_HrcCmd(ENABLE);

    /* Specify the input source of PLLA. NOTE!!! PLLA and PLLH use the same input source. */
    CLK_SetPLLSrc(CLK_PLL_SRC_HRC);
    /* PLLA configuration */
    (void)CLK_PLLxStructInit(&stcPLLAInit);
    /*
     * PLLAx(x=P, Q, R) = ((PLL_source / PLLM) * PLLN) / PLLx
     * PLLAP = (16 / 2) * 40 / 8  = 40MHz
     * PLLAQ = (16 / 2) * 40 / 10 = 32MHz
     * PLLAR = (16 / 2) * 40 / 16 = 20MHz
     */
    stcPLLAInit.u8PLLState = CLK_PLLX_ON;
    stcPLLAInit.PLLCFGR = 0UL;
    stcPLLAInit.PLLCFGR_f.PLLM = (2UL  - 1UL);
    stcPLLAInit.PLLCFGR_f.PLLN = (40UL - 1UL);
    stcPLLAInit.PLLCFGR_f.PLLR = (8UL  - 1UL);
    stcPLLAInit.PLLCFGR_f.PLLQ = (10UL - 1UL);
    stcPLLAInit.PLLCFGR_f.PLLP = (16UL - 1UL);
    (void)CLK_PLLxInit(&stcPLLAInit);
#endif
    /* 2. Specifies the clock source of ADC. */
    CLK_SetPeriClockSrc(ADC_CLK);
}

/**
 * @brief  Initializes ADC.
 * @param  None
 * @retval None
 */
static void AdcInitConfig(void)
{
    stc_adc_init_t stcAdcInit;

    /* 1. Enable ADC peripheral clock. */
    FCG_Fcg3PeriphClockCmd(ADC_PERIPH_CLK, ENABLE);

    /* 2. Modify the default value depends on the application. Not needed here. */
    (void)ADC_StructInit(&stcAdcInit);

    /* 3. Initializes ADC. */
    (void)ADC_Init(ADC_UNIT, &stcAdcInit);

    /* 4. ADC channel configuration. */
    /* 4.1 Set the ADC pin to analog input mode. */
    AdcSetPinAnalogMode();
    /* 4.2 Enable ADC channels. Call ADC_ChCmd() again to enable more channels if needed. */
    ADC_ChCmd(ADC_UNIT, ADC_SEQ, ADC_CH, ENABLE);

    /* 5. Conversion data average calculation function, if needed.
          Call ADC_ConvDataAverageChCmd() again to enable more average channels if needed. */
    ADC_ConvDataAverageConfig(ADC_UNIT, ADC_AVG_CNT8);
    ADC_ConvDataAverageChCmd(ADC_UNIT, ADC_CH, ENABLE);
}

/**
 * @brief  Use ADC in polling mode.
 * @param  None
 * @retval uint16_t u16AdcValue
 */
static uint16_t AdcPolling(void)
{
    uint16_t u16AdcValue = 0;
    int32_t iRet = LL_ERR;
    __IO uint32_t u32TimeCount = 0UL;

    /* Can ONLY start sequence A conversion.
       Sequence B needs hardware trigger to start conversion. */
    ADC_Start(ADC_UNIT);
    do {
        if (ADC_GetStatus(ADC_UNIT, ADC_EOC_FLAG) == SET) {
            ADC_ClearStatus(ADC_UNIT, ADC_EOC_FLAG);
            iRet = LL_OK;
            break;
        }
    } while (u32TimeCount++ < ADC_TIMEOUT_VAL);

    if (iRet == LL_OK) {
        /* Get any ADC value of sequence A channel that needed. */
        u16AdcValue = ADC_GetValue(ADC_UNIT, ADC_CH);
        KPrintf("The ADC value of potentiometer is %u, voltage is %u mV\r\n",
            u16AdcValue, ADC_CAL_VOL(u16AdcValue));
    } else {
        ADC_Stop(ADC_UNIT);
        KPrintf("ADC exception.\r\n");
    }

    return ADC_CAL_VOL(u16AdcValue);
}

static uint32 AdcOpen(void *dev)
{
    x_err_t ret = EOK;
    struct AdcHardwareDevice* adc_dev = (struct AdcHardwareDevice*)dev;

    AdcClockConfig();
    AdcInitConfig();

    return ret;
}

static uint32 AdcClose(void *dev)
{
    struct AdcHardwareDevice* adc_dev = (struct AdcHardwareDevice*)dev;
    CM_ADC_TypeDef *ADCx= (CM_ADC_TypeDef *)adc_dev->private_data;

    ADC_Stop(ADC_UNIT);
    ADC_DeInit(ADCx);
    
    return EOK;
}

static uint32 AdcRead(void *dev, struct BusBlockReadParam *read_param)
{
    *(uint16 *)read_param->buffer = AdcPolling();
    read_param->read_length = 2;

   return EOK;
}

static uint32 AdcDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    uint8 adc_channel;

    struct AdcDriver *adc_drv = (struct AdcDriver *)drv;
    struct AdcHardwareDevice *adc_dev = (struct AdcHardwareDevice *)adc_drv->driver.owner_bus->owner_haldev;
    struct HwAdc *adc_cfg = (struct HwAdc *)adc_dev->haldev.private_data;

    switch (configure_info->configure_cmd)
    {
        case OPE_CFG:
            adc_cfg->adc_channel = *(uint8 *)configure_info->private_data;
            if (adc_cfg->adc_channel != 1) {
                KPrintf("AdcDrvConfigure set adc channel(1) %u error!", adc_cfg->adc_channel);
                adc_cfg->adc_channel = 1;
                ret = ERROR;
            }
            break;
        default:
            break;
    }

    return ret;
}

static const struct AdcDevDone dev_done =
{
    AdcOpen,
    AdcClose,
    NONE,
    AdcRead,
};

int HwAdcInit(void)
{
    x_err_t ret = EOK;

#ifdef BSP_USING_ADC
    static struct AdcBus adc1_bus;
    static struct AdcDriver adc1_drv;
    static struct AdcHardwareDevice adc1_dev;
    static struct HwAdc adc1_cfg;

    adc1_drv.configure = AdcDrvConfigure;

    ret = AdcBusInit(&adc1_bus, ADC1_BUS_NAME);
    if (ret != EOK) {
        KPrintf("ADC1 bus init error %d\n", ret);
        return ERROR;
    }

    ret = AdcDriverInit(&adc1_drv, ADC1_DRIVER_NAME);
    if (ret != EOK) {
        KPrintf("ADC1 driver init error %d\n", ret);
        return ERROR;
    }
    ret = AdcDriverAttachToBus(ADC1_DRIVER_NAME, ADC1_BUS_NAME);
    if (ret != EOK) {
        KPrintf("ADC1 driver attach error %d\n", ret);
        return ERROR;
    }

    adc1_dev.adc_dev_done = &dev_done;
    adc1_cfg.ADCx = CM_ADC1;
    adc1_cfg.adc_channel = 1;

    ret = AdcDeviceRegister(&adc1_dev, (void *)&adc1_cfg, ADC1_DEVICE_NAME);
    if (ret != EOK) {
        KPrintf("ADC1 device register error %d\n", ret);
        return ERROR;
    }
    ret = AdcDeviceAttachToBus(ADC1_DEVICE_NAME, ADC1_BUS_NAME);
    if (ret != EOK) {
        KPrintf("ADC1 device register error %d\n", ret);
        return ERROR;
    }
#endif 

    return ret;

}





