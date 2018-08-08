
#ifndef  _ADC_H_
#define  _ADC_H_
#include "main.h"

//API
void ADC_Initialize(void);
uint8_t GetVoltageValue(void);

//内部
static uint32_t Measure_Voltage(void);


#endif
