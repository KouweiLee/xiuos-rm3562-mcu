#ifndef ADC_H
#define ADC_H

#include <ch32v20x.h>
void ADC_Function_Init(void);
u16 Get_ADC_Val(u8 ch);
u16 Get_ADC_Average(u8 ch, u8 times);
u16 Get_ConversionVal(s16 val);
#endif