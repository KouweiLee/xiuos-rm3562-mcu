/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */

#include "adc.h"
#include "debug.h"
#include "shell.h"
#include "xs_base.h"

extern s16 Calibrattion_Val;
int internal_temperature(int argc, char *argv[]) {
    u16 ADC_val;
    s32 val_mv;
    KPrintf("CalibrattionValue:%d\n", Calibrattion_Val);

    ADC_val = Get_ADC_Average(ADC_Channel_TempSensor, 10);
    Delay_Ms(500);

    ADC_val = Get_ConversionVal(ADC_val);
    KPrintf("ADC-Val:%04d\r\n", ADC_val);

    val_mv = (ADC_val * 3300 / 4096);  // ADC_val以3.3V为基准，4096为ADC分辨率

    KPrintf("mv-T-%d,%0d\n", val_mv, TempSensor_Volt_To_Temper(val_mv));

    Delay_Ms(2);
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 internal_temperature, internal_temperature,
                 test internal temperature sensor);