#include "adc.h"

/* ADC handle declaration */
ADC_HandleTypeDef AdcHandle;
/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef sConfig;
/* Variable used to get converted value */
__IO uint32_t uwADCxConvertedValue = 0;

static void Error_Handler(void)
{
  /* Infinite loop */
  while(1)
  {
  }
}

//--------------------------------------------------------------------------------------------
// ADC初始化
//--------------------------------------------------------------------------------------------
void ADC_Initialize(void)
{
	  /* ### - 1 - Initialize ADC peripheral #################################### */
  /*
   *  Instance                  = ADC1.
   *  OversamplingMode          = Disabled
   *  ClockPrescaler            = PCLK clock with no division.
   *  LowPowerAutoOff           = Disabled (For this exemple continuous mode is enabled with software start)
   *  LowPowerFrequencyMode     = Enabled (To be enabled only if ADC clock is lower than 2.8MHz) 
   *  LowPowerAutoWait          = Enabled (New conversion starts only when the previous conversion is completed)       
   *  Resolution                = 12 bit (increased to 16 bit with oversampler)
   *  SamplingTime              = 7.5 cycles od ADC clock.
   *  ScanDirection             = Upward 
   *  DataAlign                 = Right
   *  ContinuousConvMode        = Enabled
   *  DiscontinuousConvMode     = Disabled
   *  ExternalTrigConvEdge      = None (Software start)
   *  EOCSelection              = End Of Conversion event
   *  DMAContinuousRequests     = DISABLE
   */

  AdcHandle.Instance = ADC1;
  
  AdcHandle.Init.OversamplingMode      = DISABLE;
  
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
  AdcHandle.Init.LowPowerAutoOff       = DISABLE;
  AdcHandle.Init.LowPowerFrequencyMode = ENABLE;
  AdcHandle.Init.LowPowerAutoWait      = ENABLE;
    
  AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;
  AdcHandle.Init.SamplingTime          = ADC_SAMPLETIME_239CYCLES_5;
  AdcHandle.Init.ScanDirection         = ADC_SCAN_DIRECTION_UPWARD;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ContinuousConvMode    = ENABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIG_EDGE_NONE;
  AdcHandle.Init.EOCSelection          = EOC_SINGLE_CONV;
  AdcHandle.Init.DMAContinuousRequests = DISABLE;
 
  /* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* ### - 3 - Channel configuration ######################################## */
  /* Select Channel 0 to be converted */
  sConfig.Channel = ADC_CHANNEL_8;    //lgc0802
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
 /*##- 4- Start the conversion process #######################################*/  
  if(HAL_ADC_Start(&AdcHandle) != HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler();
  }
  
  /*##- 5- Wait for the end of conversion #####################################*/  
   /*  Before starting a new conversion, you need to check the current state of 
        the peripheral; if it�s busy you need to wait for the end of current
        conversion before starting a new one.
        For simplicity reasons, this example is just waiting till the end of the 
        conversion, but application may perform other tasks while conversion 
        operation is ongoing. */
	HAL_ADC_PollForConversion(&AdcHandle, 10);
}

//--------------------------------------------------------------------------------------------
//  获取当前电量值
//--------------------------------------------------------------------------------------------
uint8_t GetVoltageValue(void)
{	
	float VoltageValue;
	uint8_t  VoltageLevel = 0;
	uint32_t MeasureValue;

	ADC_Initialize();
 	HAL_Delay(10);
	MeasureValue = Measure_Voltage();
	VoltageValue = (float)MeasureValue * 3.3 / 4096;
	if(VoltageValue > (float)(3.5) || VoltageValue < (float)(0.0))	
		return VoltageLevel;/* 检测出错 */
	if(VoltageValue > (float)3.0){
		VoltageLevel = 10;
	}
	else if(VoltageValue>(float)2.90){
		VoltageLevel =9;
	}
	else if(VoltageValue>(float)2.80){
		VoltageLevel = 8;
	}
	else if(VoltageValue>(float)2.70){
		VoltageLevel = 7;
	}
	else if(VoltageValue>(float)2.60){
		VoltageLevel = 6;
	}
	else if(VoltageValue>(float)2.50){
		VoltageLevel = 5;
	}
	else if(VoltageValue>(float)2.40){
		VoltageLevel = 4;
	}
	else if(VoltageValue>(float)2.30){
		VoltageLevel = 3;
	}
	else if(VoltageValue>(float)2.20){
		VoltageLevel = 2;
	}
	else if(VoltageValue>(float)2.05){
		VoltageLevel = 1;
	}
	else if(VoltageValue>(float)0 && VoltageValue<(float)2.05){
		VoltageLevel = 0;
	}
	
	HAL_ADC_DeInit(&AdcHandle);	

	return VoltageLevel;
}

//--------------------------------------------------------------------------------------------
//  测量电压
//--------------------------------------------------------------------------------------------
static uint32_t Measure_Voltage(void)
{
	byte count = 20;
	uint32_t  MinValue = 6000;

  /* Check if the continous conversion of regular channel is finished */
	while(count--)
	if(HAL_ADC_GetState(&AdcHandle) == HAL_ADC_STATE_EOC)
	{
		/*##-5- Get the converted value of regular channel  ########################*/
		uwADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
		if(MinValue > uwADCxConvertedValue)
			MinValue = uwADCxConvertedValue;
	} 
//#if voltagebutton
//  if(VoltageCount==6)
//  {
//  	VoltageCount=0;
//  }
//  memset(&g_voltage_value, 0, sizeof(g_voltage_value));
//  FLASH_Read(FLASH_Get_Voltage_START_ADDR, sizeof(g_voltage_value), (uint32_t*)&g_voltage_value,5);
//  g_voltage_value.VoltageValue[VoltageCount] = MinValue;
//  FLASH_Write(FLASH_Get_Voltage_START_ADDR, (uint32_t*)&g_voltage_value, sizeof(g_voltage_value));
////  bochiot_init();
//	VoltageCount++;
//#endif
	return MinValue;
}

void HAL_ADC_Reset(void)
{	
	AdcHandle.State = HAL_ADC_STATE_RESET;
}


