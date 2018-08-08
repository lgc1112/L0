#include "rtc.h"

/* RTC handler declaration */
static RTC_HandleTypeDef RTCHandle;

//--------------------------------------------------------------------------------------------
//  错误处理函数
//--------------------------------------------------------------------------------------------
static void Error_Handler(void)
{
	/* Infinite loop */
	while(1)
	{
	}
}


//--------------------------------------------------------------------------------------------
//  RTC初始化
//--------------------------------------------------------------------------------------------
void RTC_Handle_init(void)
{
	/* Configure RTC */
	RTCHandle.Instance = RTC;
	/* Set the RTC time base to 1s */
	/* Configure RTC prescaler and RTC data registers as follow:
	- Hour Format = Format 24
	- Asynch Prediv = Value according to source clock
	- Synch Prediv = Value according to source clock
	- OutPut = Output Disable
	- OutPutPolarity = High Polarity
	- OutPutType = Open Drain */
	RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
	RTCHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
	RTCHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
	RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
	RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	
	if(HAL_RTC_Init(&RTCHandle) != HAL_OK){
		/* Infinite loop */
		Error_Handler();
	}

}


//--------------------------------------------------------------------------------------------
// 获取RTC实时时间API
//--------------------------------------------------------------------------------------------
void get_real_tim(uint8_t *time)
{
	RTC_DateTypeDef sdatestructureget;
    RTC_TimeTypeDef stimestructureget;
	
	HAL_RTC_GetTime(&RTCHandle, &stimestructureget, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&RTCHandle, &sdatestructureget, RTC_FORMAT_BCD);
	
	time[0] = sdatestructureget.Year;
	time[1] = sdatestructureget.Month;
	time[2] = sdatestructureget.Date;
	time[3] = stimestructureget.Hours;
	time[4] = stimestructureget.Minutes;
	time[5] = stimestructureget.Seconds;
}


//--------------------------------------------------------------------------------------------
//  Section xxx: 更新系统时间
//--------------------------------------------------------------------------------------------
void RTC_TimeStampConfig(uint8_t* time)
{
	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;
	
	sdatestructure.Year = time[0];
	sdatestructure.Month = time[1];
	sdatestructure.Date = time[2];
	sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

	if(HAL_RTC_SetDate(&RTCHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK){
		/* Initialization Error */
		Error_Handler(); 
	} 

	stimestructure.Hours = time[3];
	stimestructure.Minutes = time[4];
	stimestructure.Seconds = time[5];
	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if(HAL_RTC_SetTime(&RTCHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK){
		/* Initialization Error */
		Error_Handler(); 
	}
}


//--------------------------------------------------------------------------------------------
//  请求时间同步API
//--------------------------------------------------------------------------------------------
int time_request()
{
	uint8_t syc_msg[25];
	mes_to_updata_t mes_to_gateway;
	
	mes_to_gateway.protocol_type[0] = 0xb0;
	mes_to_gateway.protocol_type[1] = 0xc0;
	mes_to_gateway.datalength[0] = 0x00; 
	mes_to_gateway.datalength[1] = 0x15; //25bytes - 4
	mes_to_gateway.nodemac[0] = 0x1a;
	mes_to_gateway.nodemac[1] = 0xaa;	 
	mes_to_gateway.nodemac[2] = DeviceType;
	mes_to_gateway.nodemac[3] = DevcMAC4;
	mes_to_gateway.nodemac[4] = DevcMAC5;
	mes_to_gateway.nodemac[5] = DevcMAC6;
	mes_to_gateway.protocol_ID[0] = 0x11;
	mes_to_gateway.protocol_ID[1] = 0x22;
	mes_to_gateway.cmd = cmd_synchronize_tim;
	mes_to_gateway.payloadlength = 0x03; //3bytes
	mes_to_gateway.device_type = 0x02; //��
	//memcpy(g_mes_to_gateway.gwmac,g_endevice_info.server_addr,6);
	memset(&mes_to_gateway.gwmac,0,sizeof(mes_to_gateway.gwmac));//lgc 0726
	memcpy(&syc_msg,&mes_to_gateway,sizeof(syc_msg)-2);
	syc_msg[23] = bochiot_check_str(syc_msg,23);
	syc_msg[24] = 0xFF;	
	
	bochiot_send_msg((uint8*)&syc_msg,25);		
	HAL_Delay(1500);	
	return LORA_RX();
}




RTC_HandleTypeDef *get_RTC_Handle()
{
	return &RTCHandle;

}





















