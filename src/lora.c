#include "lora.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "bochiot_protocol.h"
//20151008
//*********************************************************
//PIN IO mapping
//*********************************************************
//		 MCU     Bochi Lora module 
// 0.   PORChk---RST
// 1.	 nCS-----nCS
// 2.	 SCK-----SCK
// 3.	MOSI-----MOSI
// 4.	MISO-----MISO
//*********************************************************


#define RX_BUF_LEN	128

#if 1
#define DEBUG_BOARD		0
#define SPI_TX_MODE	0
#define SPI_RX_MODE	1

#if DEBUG_BOARD
#define FEM_CPS_PORT	GPIOC
#define FEM_CPS_PIN     GPIO_PIN_0

#define FEM_CPS_TX		FEM_CPS_PORT->BRR = FEM_CPS_PIN	  //set 0
#define FEM_CPS_RX		FEM_CPS_PORT->BSRR= FEM_CPS_PIN   //set 1
#else
//jking
#define SPI1_FEM_CPS_PORT	GPIOC
#define RFSW1_CTRL1_PIN     GPIO_PIN_4
#define RFSW1_CTRL2_PIN     GPIO_PIN_5

#define SPI2_FEM_CPS_PORT	GPIOB
#define RFSW2_CTRL1_PIN     GPIO_PIN_0
#define RFSW2_CTRL2_PIN     GPIO_PIN_1

#define SPIx_FEM_CPS_PORT	GPIOB
#define RFSWx_CTRL1_PIN     GPIO_PIN_6
#define RFSWx_CTRL2_PIN     GPIO_PIN_7

#define LORA1_RST_PORT		GPIOC
#define RESET1_PIN     		GPIO_PIN_0
#define LORA2_RST_PORT		GPIOC
#define RESET2_PIN     		GPIO_PIN_1
#define LORA3_RST_PORT		GPIOC
#define RESET3_PIN     		GPIO_PIN_2

#endif

//JinPeng
#define Rx_Timeout 1000

extern volatile unsigned char Value_LR_RegFrMsb;
extern volatile unsigned char Value_LR_RegFrMid;
extern volatile unsigned char Value_LR_RegFrLsb;
extern volatile unsigned char Value_LR_RegModemConfig1;
extern volatile unsigned char Value_LR_RegModemConfig2;
extern volatile unsigned char Value_LR_RegModemConfig3;
extern volatile unsigned char Value_LR_RegPaConfig;
extern volatile unsigned char Value_LR_RegPreambleMsb;
extern volatile unsigned char Value_LR_RegPreambleLsb;

void RFSW_SET_PIN(SPI_HandleTypeDef *hspi,int mode)
{

#if DEBUG_BOARD
	if(SPI_TX_MODE==mode)
	{
		FEM_CPS_TX;	
	}
	else 
	{
		FEM_CPS_RX;		
	}
#else
	if((uint32_t)(hspi->Instance)==(uint32_t)SPI1)
	{
		if(SPI_RX_MODE==mode)
		{
			SPI1_FEM_CPS_PORT->BSRR = RFSW1_CTRL1_PIN;
			SPI1_FEM_CPS_PORT->BRR = RFSW1_CTRL2_PIN;			
		}
		else 
		{
			SPI1_FEM_CPS_PORT->BRR = RFSW1_CTRL1_PIN;
			SPI1_FEM_CPS_PORT->BSRR = RFSW1_CTRL2_PIN;			
		}
	}
	else if((uint32_t)(hspi->Instance)==(uint32_t)SPI2)
	{
		if(SPI_RX_MODE==mode)
		{
			SPI2_FEM_CPS_PORT->BSRR = RFSW2_CTRL1_PIN;
			SPI2_FEM_CPS_PORT->BRR = RFSW2_CTRL2_PIN;			
		}
		else 
		{
			SPI2_FEM_CPS_PORT->BRR = RFSW2_CTRL1_PIN;
			SPI2_FEM_CPS_PORT->BSRR = RFSW2_CTRL2_PIN;			
		}
	}
	else
	{
		if(SPI_RX_MODE==mode)
		{
			SPIx_FEM_CPS_PORT->BSRR = RFSWx_CTRL1_PIN;
			SPIx_FEM_CPS_PORT->BRR = RFSWx_CTRL2_PIN;			
		}
		else 
		{
			SPIx_FEM_CPS_PORT->BRR = RFSWx_CTRL1_PIN;
			SPIx_FEM_CPS_PORT->BSRR = RFSWx_CTRL2_PIN;			
		}
	}
#endif
}

/******************************************************************************
**函数名称：Lora_FEM_CPS_init
**函数功能：使用Lora发送或者接收前，必须调用该函数初始化，初始化TX与RX的开关引脚
**输入参数：无 
**输出参数：无
******************************************************************************/
void Lora_FEM_CPS_init(void)
{
#if DEBUG_BOARD
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode= GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin= FEM_CPS_PIN;

	HAL_GPIO_Init(FEM_CPS_PORT, &GPIO_InitStructure);
#else
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode= GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin= RFSW1_CTRL1_PIN;
	HAL_GPIO_Init(SPI1_FEM_CPS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode= GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin= RFSW1_CTRL2_PIN;
	HAL_GPIO_Init(SPI1_FEM_CPS_PORT, &GPIO_InitStructure);

	////////////////////////////////////	
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode= GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin= RFSW2_CTRL1_PIN;
	HAL_GPIO_Init(SPI2_FEM_CPS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode= GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin= RFSW2_CTRL2_PIN;
	HAL_GPIO_Init(SPI2_FEM_CPS_PORT, &GPIO_InitStructure);
	
	////////////////////////////////////	
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode= GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin= RFSWx_CTRL1_PIN;
	HAL_GPIO_Init(SPIx_FEM_CPS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode= GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin= RFSWx_CTRL2_PIN;
	HAL_GPIO_Init(SPIx_FEM_CPS_PORT, &GPIO_InitStructure);

//////////////////reset pin
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode= GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin= RESET1_PIN;
	HAL_GPIO_Init(LORA1_RST_PORT, &GPIO_InitStructure);
	HAL_GPIO_WritePin(LORA1_RST_PORT,RESET1_PIN, GPIO_PIN_SET); 
	
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode= GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin= RESET2_PIN;
	HAL_GPIO_Init(LORA2_RST_PORT, &GPIO_InitStructure);
	HAL_GPIO_WritePin(LORA2_RST_PORT,RESET2_PIN, GPIO_PIN_SET); 
	
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode= GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin= RESET3_PIN;
	HAL_GPIO_Init(LORA3_RST_PORT, &GPIO_InitStructure);
	HAL_GPIO_WritePin(LORA3_RST_PORT,RESET3_PIN, GPIO_PIN_SET); 
#endif
}
#endif


//申请变量

byte 	SysTime;					//16ms定时
byte 	KeyTime;					//按键扫描定时
byte 	TxLedTime;					//TxLed闪烁动态
byte 	TxLedCnt; 					//TxLed计数
byte 	TxLimtTime;					//每包数据超时
byte 	RxLimtTime;					//	 
byte 	ChkSettingTime;				//检查时间
byte 	ChkRxTime;					//检查RxMode
byte 	SecTime;					//
byte 	Error2_F;	 				//错误代码
byte 	RxData[RX_BUF_LEN];	   				//接收数据
byte 	TxBuf[64]; 	   				//发送数据缓冲



//SPI 接口操作函数
extern void SPIx_Write(SPI_HandleTypeDef *hspi, char* Value, byte size);
extern uint32_t SPIx_Read(SPI_HandleTypeDef *hspi, char* pData,uint8 read_len);
extern void SPI_SNN_On(void);

extern void SPI_SNN_Off(void);

extern void LORA_RXLED_On(void);

extern void LORA_RXLED_Off(void);

extern void Delay_us(uint32_t us);

extern void sx1276_burst_read(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t *buf, int len);

/******************************************************************************
**函数名称：LoraRegRead
**函数功能：读出指定的寄存器地址的数据
**输入参数：WrPara
**输出参数：无
******************************************************************************/
void LoraRegRead(SPI_HandleTypeDef *SpiHandle, byte adr, char* pData, byte read_len)
{	
	byte  cData[128];		
	
	if(NULL == SpiHandle)
	{
	  	return ;
	}	
	memset(cData, 0, 128);
	cData[0] = adr;
	SPIx_Write(SpiHandle, (char*)cData, 1);
	memset(RxData, 0, RX_BUF_LEN);
	SPIx_Read(SpiHandle, (char*)RxData,read_len);  //jking
	strcpy(pData, (char*)RxData);
	return;
}
/******************************************************************************
	**函数名称：LoraRegWrite
	**函数功能：写入一个16数据（高8位寄存器地址，低8位数据)到指定寄存器
	**输入参数：WrPara
	**输出参数：无
	******************************************************************************/
void LoraRegWrite(SPI_HandleTypeDef *SpiHandle,uint8_t addr, uint8_t *buffer, uint8_t size)
{	
	byte  cData[128];	
  
	if(NULL == SpiHandle)
	{
	  	return ;
	}

	cData[0] = addr + 0x80;
	if(size>0)
	{
		memcpy(cData+1,buffer,size);
	}
	SPIx_Write(SpiHandle, (char*)cData, size+1);
	SPIx_Read(SpiHandle,(char*)RxData,2);  //jking
	memset(RxData,0,RX_BUF_LEN);
	LoraRegRead(SpiHandle,addr,(char*)RxData,2);
	return;
}

	
		
	/******************************************************************************
	**函数名称：SPIBurstRead
	**函数功能：SPI连续读取模式
	**输入参数：adr——读取地址
	**			ptr——存储数据指针
	**			length 读取长度
	**输出参数：无，数据存在ptr中
	******************************************************************************/
void LoraBurstRead(SPI_HandleTypeDef *SpiHandle, byte adr, byte *ptr, byte length)
{		
	if(length<=1)			//读取长度必须大于1
	{
		return;
	}
	else
	{	
		LoraRegRead(SpiHandle, adr, (char*)ptr,length);		//读取地址	 
	
	}
	 
		
}
	
	/******************************************************************************
	**函数名称：SPIBurstWrite
	**函数功能：SPI连续写入模式
	**输入参数：adr——写入地址
	**			ptr——存储数据指针
	**			length 写入长度
	**输出参数：无
	******************************************************************************/
void LoraBurstWrite(SPI_HandleTypeDef *SpiHandle, byte adr, byte *ptr, byte length)
{ 	
	
	if(length<=1)			//读取长度不为0或1
	{
		return;
	}
	else	
	{								
		//连续写 写FiFo寄存器
		LoraRegWrite(SpiHandle,0x0000,ptr,length);

	}
}
	
	
byte ConstrueSPIData(char *pSPIdata)
{
	if(NULL == pSPIdata)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
	
	


/******************************************************************************
**函数名称：Lora_Sleep
**函数功能：Lora睡眠
**输入参数：无
**输出参数：无
******************************************************************************/
void Lora_Sleep(SPI_HandleTypeDef *SpiHandle)
{
	uint8 data = VALUE_RegOpMode_SLEEP;
	LoraRegWrite(SpiHandle, LR_RegOpMode,&data,1);

}

/******************************************************************************
**函数名称：Lora_EntryLoRa
**函数功能：Lora进入LongRange
**输入参数：无
**输出参数：无
******************************************************************************/
void Lora_EntryLoRa(SPI_HandleTypeDef *SpiHandle)
{

	uint8 data = VALUE_RegOpMode_LORA;
	LoraRegWrite(SpiHandle,LR_RegOpMode,&data,1);
}

/******************************************************************************
**函数名称：Lora_ClearIrq
**函数功能：清除所有Irq
**输入参数：无
**输出参数：无
******************************************************************************/
void Lora_ClearIrq(SPI_HandleTypeDef *SpiHandle)
{
  //写1清除中断  
  uint8 data = VALUE_LR_RegIrqFlags;
  LoraRegWrite(SpiHandle,LR_RegIrqFlags,&data,1);
	LoraRegRead(SpiHandle,LR_RegIrqFlags,(char*)&data,1);
}


/******************************************************************************
**函数名称：Lora_Standby
**函数功能：Lora设置进入Standby
**输入参数：无
**输出参数：无
******************************************************************************/
void Lora_Standby(SPI_HandleTypeDef *SpiHandle)
{

 	//Low Frequency and standby
 	uint8 data = VALUE_RegOpMode_STANDBY;
	LoraRegWrite(SpiHandle,LR_RegOpMode,&data,1);

}


/******************************************************************************				
**函数名称：Lora_Config
**函数功能：Sleep模块，然后配置进入LongRangeMode
**输入参数：无
**输出参数：无
//基本配置寄存器信息:

0x06 0x07 0x08 0x09 0x0B  0x0C 0x1D 0x1E  0x1F 0x20 0x21 0x41 0x01   


******************************************************************************/

void Lora_Config(SPI_HandleTypeDef *SpiHandle)
{

	uint8 data;

	//进入休眠模式
	Lora_Sleep(SpiHandle);
	
	//进入LongRangeMode
	Lora_EntryLoRa(SpiHandle);	

#if 1
	//配置频率 //434MHz   
	data = Value_LR_RegFrMsb;
	LoraRegWrite(SpiHandle, LR_RegFrMsb, &data,1);

	data = Value_LR_RegFrMid;
	LoraRegWrite(SpiHandle, LR_RegFrMid, &data,1);
	
	data = Value_LR_RegFrLsb;
	LoraRegWrite(SpiHandle, LR_RegFrLsb,&data,1);



	//配置RF基本参数
	//Disable
	data = VALUE_LR_RegOcp;
	LoraRegWrite(SpiHandle,LR_RegOcp,&data,1);
#endif 
	//RegLna Inc
	data = VALUE_LR_RegLna;
	LoraRegWrite(SpiHandle,LR_RegLna,&data,1);

	//Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
	data = Value_LR_RegModemConfig1;
	LoraRegWrite(SpiHandle,LR_RegModemConfig1,&data,1);

	//SFactor &  LNA gain set by the internal AGC loop  
	data = Value_LR_RegModemConfig2;
	LoraRegWrite(SpiHandle,LR_RegModemConfig2,&data,1);
#if 0
	//RegSymbTimeoutLsb Timeout = 0x3FF(Max)
	data = VALUE_LR_RegSymbTimeoutLsb;
	LoraRegWrite(SpiHandle,LR_RegSymbTimeoutLsb,&data,1);

	//RegPreambleMsb	高八位
	data = 0xff;//VALUE_LR_RegPreambleMsb;
	LoraRegWrite(SpiHandle,LR_RegPreambleMsb,&data,1);

	//RegPreambleLsb	12+4=16Byte Preamble   8+4    低八位 
	data = 0xf0;//VALUE_LR_RegPreambleLsb;
	LoraRegWrite(SpiHandle,LR_RegPreambleLsb,&data,1);
#endif

#if 0
	data = VALUE_LR_RegDIOMAPPING2;
	LoraRegWrite(SpiHandle,LR_RegDIOMAPPING2,&data,1);


	//20dBm
	data = VALUE_LR_RegPaConfig;
	LoraRegWrite(SpiHandle,LR_RegPaConfig,&data,1);


	//进入Standby后进行配置，有些配置需要在Standby or Sleep中配置
	Lora_Standby(SpiHandle);	
#endif

}

void Lora_Config_Init(SPI_HandleTypeDef *SpiHandle)
{

	uint8 data;
	char pdata[32];

	//进入休眠模式
	Lora_Sleep(SpiHandle);
	LoraRegRead(SpiHandle,LR_RegOpMode,pdata,2);
	
	//进入LongRangeMode
	Lora_EntryLoRa(SpiHandle);	

	//配置频率 //434MHz   
	//配置寄存器 0X06---0X0F
	data = Value_LR_RegFrMsb;
	LoraRegWrite(SpiHandle, LR_RegFrMsb, &data,1);	

	data = Value_LR_RegFrMid;
	LoraRegWrite(SpiHandle, LR_RegFrMid, &data,1);
	
	data = Value_LR_RegFrLsb;
	LoraRegWrite(SpiHandle, LR_RegFrLsb,&data,1);

	data = Value_LR_RegPaConfig;
	LoraRegWrite(SpiHandle, LR_RegPaConfig,&data,1);

	//0X0A
	data = VALUE_LR_RegPaRamp;
	LoraRegWrite(SpiHandle, LR_RegPaRamp,&data,1);

	//0X0B
	data = VALUE_LR_RegOcp;
	LoraRegWrite(SpiHandle, LR_RegOcp,&data,1);

	//0X0C
	data = VALUE_LR_RegLna;
	LoraRegWrite(SpiHandle, LR_RegLna,&data,1);

	//0X0D
	data = VALUE_LR_RegFifoAddrPtr;
	LoraRegWrite(SpiHandle, LR_RegFifoAddrPtr,&data,1);

	//0X0E
	data = VALUE_LR_RegFifoTxBaseAddr;
	LoraRegWrite(SpiHandle, LR_RegFifoTxBaseAddr,&data,1);

	//0X0F
	data = VALUE_LR_RegFifoRxBaseAddr;
	LoraRegWrite(SpiHandle, LR_RegFifoRxBaseAddr,&data,1);

	
	//配置寄存器 0X10---0X1F		
	//0X11
	data = VALUE_LR_RegIrqFlagsMask;
	LoraRegWrite(SpiHandle, LR_RegIrqFlagsMask,&data,1);
	
	//0X12
	data = VALUE_LR_RegIrqFlags;
	LoraRegWrite(SpiHandle, LR_RegIrqFlags,&data,1);

	//0X1D
	data = Value_LR_RegModemConfig1;
	LoraRegWrite(SpiHandle, LR_RegModemConfig1,&data,1);

	//0X1E
	data = Value_LR_RegModemConfig2;
	LoraRegWrite(SpiHandle, LR_RegModemConfig2,&data,1);

	//0X1F
	data = VALUE_LR_RegSymbTimeoutLsb;
	LoraRegWrite(SpiHandle, LR_RegSymbTimeoutLsb,&data,1);

	//配置寄存器 0X20---0X2F		
	//0X20	RegPreambleMsb	
	data = Value_LR_RegPreambleMsb;//VALUE_LR_RegPreambleMsb;
	LoraRegWrite(SpiHandle,LR_RegPreambleMsb,&data,1);

	//0X21	RegPreambleLsb	12+4=16Byte Preamble
	data =  Value_LR_RegPreambleLsb;//VALUE_LR_RegPreambleLsb;
	LoraRegWrite(SpiHandle,LR_RegPreambleLsb,&data,1);

	//0X22	
	data = VALUE_LR_RegPayloadLength;
	LoraRegWrite(SpiHandle,LR_RegPayloadLength,&data,1);

	//0X23	
	memset(pdata,0,32); 
	data = VALUE_LR_RegMaxPayloadLength;
	LoraRegWrite(SpiHandle,LR_RegMaxPayloadLength,&data,1);

	//0X24	
	memset(pdata,0,32); 
	data = VALUE_LR_RegHopPeriod;
	LoraRegWrite(SpiHandle,LR_RegHopPeriod,&data,1);

	//0X26	
	memset(pdata,0,32); 
	data = Value_LR_RegModemConfig3;
	LoraRegWrite(SpiHandle,LR_RegModemConfig3,&data,1);


	//配置寄存器 0X40---0X4D		
	//0X40	
	memset(pdata,0,32); 
	data = VALUE_LR_RegDIOMAPPING1;
	LoraRegWrite(SpiHandle,LR_RegDIOMAPPING1,&data,1);

	//0X41	
	data = VALUE_LR_RegDIOMAPPING2;
	LoraRegWrite(SpiHandle,LR_RegDIOMAPPING2,&data,1);

	//0X4B	
	data = VALUE_LR_RegTCXO;
	LoraRegWrite(SpiHandle,LR_RegTCXO,&data,1);

	//配置寄存器 0X61---0X64		
	//0X61	
	data = VALUE_LR_RegAGCREF;
	LoraRegWrite(SpiHandle,LR_RegAGCREF,&data,1);

	//0X62	
	data = VALUE_LR_RegAGCTHRESH1;
	LoraRegWrite(SpiHandle,LR_RegAGCTHRESH1,&data,1);

	//0X63	
	data = VALUE_LR_RegAGCTHRESH2;
	LoraRegWrite(SpiHandle,LR_RegAGCTHRESH2,&data,1);

	//0X64	
	data = VALUE_LR_RegAGCTHRESH3;
	LoraRegWrite(SpiHandle,LR_RegAGCTHRESH3,&data,1);

	
	//配置寄存器 0X70		
	//0X70	 
	data = VALUE_LR_RegPll;
	LoraRegWrite(SpiHandle,LR_RegPll,&data,1);
	
	//进入Standby后进行配置，有些配置需要在Standby or Sleep中配置
	Lora_Standby(SpiHandle);	

}

/**************************************************************
**函数名称: Lora_CAD_config
**函数功能：启用CAD信道检测功能
**输入参数：无
**输出参数：无
**************************************************************/
void Lora_CadConfig(SPI_HandleTypeDef *SpiHandle)
{
	uint8 parameter = 0;
	char pdata[32];

	Lora_Sleep(SpiHandle);


#if 1
		//配置频率 //434MHz   
	parameter =Value_LR_RegFrMsb;
	LoraRegWrite(SpiHandle, LR_RegFrMsb, &parameter,1);
	parameter = 0;
	LoraRegRead(SpiHandle, LR_RegFrMsb, (char *)&parameter,1);
	parameter = Value_LR_RegFrMid;
	LoraRegWrite(SpiHandle, LR_RegFrMid, &parameter,1);
	
	parameter = Value_LR_RegFrLsb;
	LoraRegWrite(SpiHandle, LR_RegFrLsb,&parameter,1);
#endif


	//配置RF基本参数
	//Disable
	parameter = VALUE_LR_RegOcp;
	LoraRegWrite(SpiHandle,LR_RegOcp,&parameter,1);
	
		//RegLna Inc
	parameter = VALUE_LR_RegLna;
	LoraRegWrite(SpiHandle,LR_RegLna,&parameter,1);

	//Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
	parameter = Value_LR_RegModemConfig1;
	LoraRegWrite(SpiHandle,LR_RegModemConfig1,&parameter,1);
	
	//SFactor &  LNA gain set by the internal AGC loop  
	parameter = Value_LR_RegModemConfig2;
	LoraRegWrite(SpiHandle,LR_RegModemConfig2,&parameter,1);
	
//		//RegPreambleMsb	高八位
//	parameter = 0xff;//VALUE_LR_RegPreambleMsb;
//	LoraRegWrite(SpiHandle,LR_RegPreambleMsb,&parameter,1);

//	//RegPreambleLsb	12+4=16Byte Preamble   8+4    低八位 
//	parameter = 0xf0;//VALUE_LR_RegPreambleLsb_Tx;
//	LoraRegWrite(SpiHandle,LR_RegPreambleLsb,&parameter,1);


	Lora_Standby(SpiHandle);
	LoraRegRead(SpiHandle,LR_RegOpMode,(char*)pdata,2);

	Lora_ClearIrq(SpiHandle);
	LoraRegRead(SpiHandle,LR_RegIrqFlags,pdata,1);
	//仅打开CAD终断
	parameter = VALUE_LR_RegIrqFlagsMask_CAD;
	LoraRegWrite(SpiHandle, LR_RegIrqFlagsMask,&parameter,1);
//	LoraRegRead(SpiHandle,LR_RegIrqFlagsMask,pdata,1);
	
	//DIO0=10, DIO1=10, DIO2=00, DIO3=10
	parameter = VALUE_LR_RegDIOMAPPING1_CAD; 
	LoraRegWrite(SpiHandle, LR_RegDIOMAPPING1,&parameter,1);
//	LoraRegRead(SpiHandle,LR_RegDIOMAPPING1,pdata,1);
	
	//进入CAD信道检测模式
	parameter = VALUE_RegOpMode_CAD;
	LoraRegWrite(SpiHandle, LR_RegOpMode,&parameter,1);
//	LoraRegRead(SpiHandle, LR_RegOpMode, pdata, 1);	 
}


/******************************************************************************
**函数名称：Lora_EntryRx
**函数功能：Lora设置进入接收
**输入参数：无
**输出参数：0——状态异常

//RX 配置的寄存器信息:

	0x01 0x0D 0x11 0x21 0x22 0x22 0x24 0x26 0x40 0x4D

******************************************************************************/
byte Lora_EntryRx(SPI_HandleTypeDef *SpiHandle)
{	

	uint8 parameter = 0;
	char data[32];
	memset(data, 0, 32);
	
	//检测接收中断信号电平
		//Lora_Config(SpiHandle);						//基本配置
		Lora_Config_Init(SpiHandle);			
	#if 1
		LoraRegRead(SpiHandle, LR_RegModemConfig3, data,2);
		//parameter =data[0]+ VALUE_LR_RegModemConfig3;
		parameter =data[0]+ 0x08;   //add when do testing by liuxiang
		//parameter =data[0]+ 0x00;   //add when do testing by liuxiang
		LoraRegWrite(SpiHandle,LR_RegModemConfig3,&parameter,1);
		//LoraRegRead(SpiHandle, LR_RegModemConfig3, data,2);

		//delete
		//Normal and Rx
		//parameter = VALUE_LR_RegPADAC_RX;
		//LoraRegWrite(SpiHandle,LR_RegPADAC,&parameter,1);
		
		//RegHopPeriod NO FHSS	
		//parameter = VALUE_LR_RegHopPeriod;
		//LoraRegWrite(SpiHandle, LR_RegHopPeriod,&parameter,1);

		
		//end delete
		
		//DIO0=00, DIO1=00, DIO2=00, DIO3=10  
		parameter = VALUE_LR_RegDIOMAPPING1_RX;
		LoraRegWrite(SpiHandle, LR_RegDIOMAPPING1,&parameter,1);
		
		//RegPayloadLength  设定允许最大长度 for Implicit Mode 必须	
		//delete 
		//parameter = VALUE_LR_RegPayloadLength;
		//LoraRegWrite(SpiHandle, LR_RegPayloadLength,&parameter,1);
		//end delete

		Lora_ClearIrq(SpiHandle);
		
		//仅打开RxDone中断 & Timeout & CRC
		parameter = VALUE_LR_RegIrqFlagsMask_CRT;
		LoraRegWrite(SpiHandle, LR_RegIrqFlagsMask,&parameter,1);//仅打开RxDone中断 & Timeout

		//Continuous 
		LoraRegRead(SpiHandle, (byte)(LR_RegFifoRxBaseAddr), data,2);	//Read RxBaseAddr
		LoraRegWrite(SpiHandle, LR_RegFifoAddrPtr,(uint8*)&data[0],1);

		//Continuous Rx Mode and Low frequency	
		parameter = VALUE_RegOpMode_ContinuousRx;
		LoraRegWrite(SpiHandle, LR_RegOpMode,&parameter,1);

		//delete 
		//parameter = 0x8;  //add by liuxiang
		//parameter = VALUE_LR_RegPreambleLsb; 
		//LoraRegWrite(SpiHandle, LR_RegPreambleLsb,&parameter,1);
		//parameter = 0x1;
		//LoraRegWrite(SpiHandle, LR_RegPayloadLength,&parameter,1);
		//parameter = VALUE_LR_RegMaxPayloadLength;
		//LoraRegWrite(SpiHandle, LR_RegMaxPayloadLength,&parameter,1);
		//end delete

		memset(data,0,32);
		for(SysTime=0;SysTime<3;SysTime++)	//等待状态稳定
		{  
			LoraRegRead(SpiHandle, (byte)(LR_RegModemStat), data,2);		
			if((data[0]&0x04)==0x04)		//Rx-on going RegModemStat rx进行中
			{
				break;				//状态稳定
			}
		} 	 

		if(SysTime>=3)	
			return(0);				//返回异常
		else
			return(1);				//返回正常
	#endif
	
}

/******************************************************************************
**函数名称：Lora_RxPacket
**函数功能：接收一包数据
**输入参数：无
**输出参数：1——接收成功
**          0——接收失败
******************************************************************************/
byte Lora_RxPacket(SPI_HandleTypeDef *SpiHandle, char* pData)
{
	RFSW_SET_PIN(SpiHandle,SPI_RX_MODE);//FEM_CPS_RX;  //必须调用，切换到接收模式
	byte packet_size = 0;
	uint32_t tickstart = 0;
	int count=0;

	tickstart = HAL_GetTick();
	while(HAL_GetTick() - tickstart<3000)
	{	
		count++;
		//LoraBurstRead(SpiHandle, LR_RegFifo, RxData,5);
		
		//检测接收中断信号电平
		LoraRegRead(SpiHandle, (byte)(LR_RegIrqFlags), (char*)RxData,2);
		
		if((RxData[0] & 0x40)==0x40) //rxDone
		{ 
//			LORA_RXLED_Off();
			if((RxData[0] & 0x20)==0x20) //CRCError
			{	
				Lora_ClearIrq(SpiHandle);
				continue;
			}
			memset(RxData, 0, RX_BUF_LEN); 		
			//Continuous
			LoraRegRead(SpiHandle, (byte)(LR_RegFifoRxCurrentaddr), (char*)RxData,2);	//last packet addr
			LoraRegWrite(SpiHandle,LR_RegFifoAddrPtr,(uint8*)&RxData[0],1);
			
			memset(RxData, 0, RX_BUF_LEN); 	
			LoraRegRead(SpiHandle, LR_RegPktSnrValue, (char*)RxData, 1);
			
			//读取RegNbRxByte 即Number for received bytes
			LoraRegRead(SpiHandle, (byte)(LR_RegRxNbBytes), (char*)&packet_size,2);	

			memset(RxData, 0, RX_BUF_LEN);
			LoraBurstRead(SpiHandle, LR_RegFifo, RxData,packet_size);
			
			memcpy(pData, RxData, packet_size);

			Lora_ClearIrq(SpiHandle);

			return packet_size;
		}
		else if((RxData[0] & 0x80)==0x80)
		{
			Lora_ClearIrq(SpiHandle);
			continue;
		}
	}
	return 0;
}

/******************************************************************************
**函数名称：Lora_EntryTx
**函数功能：Lora设置进入发送指定的数据
**输入参数：无
**输出参数：0——状态异常
//TX 配置的寄存器信息:

	0x00 0x01 0x0D  0x0E 0x11  0x22  0x24  0x40  0x4D 

******************************************************************************/
byte Lora_EntryTx(SPI_HandleTypeDef *SpiHandle, char* pData, byte iSize)
{
 byte data;
 char TxFlgs[32];
// signed char VL_Rssi;
 srand(75);

	if(NULL == pData)
 {
 	return 0;
 }

 //基本配置(434)
 //Lora_Config(SpiHandle);	
 Lora_Config_Init(SpiHandle);

 //data = VALUE_LR_RegFrLsb;
 //LoraRegWrite(SpiHandle, LR_RegFrLsb,&data,1); 
 #if 1
 //Tx for 20dBm 
 data = VALUE_LR_RegPADAC_TX;
 LoraRegWrite(SpiHandle, LR_RegPADAC,&data,1);

 memset(RxData, 0, RX_BUF_LEN);
 LoraRegRead(SpiHandle, LR_RegPADAC,(char*)RxData,2);	
#endif 

 //RegHopPeriod NO FHSS
 data = VALUE_LR_RegHopPeriod;
 LoraRegWrite(SpiHandle, LR_RegHopPeriod,&data,1);
 memset(RxData, 0, RX_BUF_LEN);
 LoraRegRead(SpiHandle, LR_RegHopPeriod,(char*)RxData,2);	
 


 data = iSize;
 LoraRegWrite(SpiHandle, LR_RegPayloadLength,&data,1);	
 memset(RxData, 0, RX_BUF_LEN);
 LoraRegRead(SpiHandle, LR_RegPayloadLength,(char*)RxData,2);	

 memset(RxData, 0, RX_BUF_LEN);
 LoraRegRead(SpiHandle,(byte)(LR_RegFifoTxBaseAddr), (char*)RxData,2);	//RegFiFoTxBaseAddr
 //RegFifoAddrPtr 设置指针
 LoraRegWrite(SpiHandle, LR_RegFifoAddrPtr,(uint8*)RxData[0],1);

RxData[0] = 0;
 memset(RxData, 0, RX_BUF_LEN);
 RxData[0] = 0x80;  //add by liuxiang
LoraRegWrite(SpiHandle, LR_RegFifoTxBaseAddr,(uint8*)&RxData[0],1);
 memset(RxData, 0, RX_BUF_LEN);
LoraRegRead(SpiHandle,(byte)(LR_RegFifoTxBaseAddr), (char*)RxData,2);
LoraRegWrite(SpiHandle, LR_RegFifoAddrPtr,(uint8*)&RxData[0],1);
 memset(RxData, 0, RX_BUF_LEN);
LoraRegRead(SpiHandle,(byte)(LR_RegFifoAddrPtr), (char*)RxData,2);

__disable_irq();
 LoraBurstWrite(SpiHandle, 0x00, (byte *)pData, iSize);
 __enable_irq();
 

	
 //LORA_RXLED_Off();
 Lora_Standby(SpiHandle);	
 
 //DIO0=01, DIO1=00, DIO2=00, DIO3=01
 data  =VALUE_LR_RegDIOMAPPING1_TX;
 LoraRegWrite(SpiHandle, LR_RegDIOMAPPING1,&data,1);
 memset(RxData, 0, RX_BUF_LEN);
 LoraRegRead(SpiHandle, LR_RegDIOMAPPING1,(char*)RxData,2);	
 Lora_ClearIrq(SpiHandle);

 //仅打开TxDone中断
 data = VALUE_LR_RegIrqFlagsMask_TX;
 LoraRegWrite(SpiHandle, LR_RegIrqFlagsMask,&data,1);	 
 memset(RxData, 0, RX_BUF_LEN);
 LoraRegRead(SpiHandle, LR_RegIrqFlagsMask,(char*)RxData,2);	
 
 
 //Tx Mode  and Low frequency mode
 memset(TxFlgs, 0, 32); 
 memset(RxData, 0, RX_BUF_LEN);
  LoraRegRead(SpiHandle, LR_RegOpMode,(char*)RxData,2); 
 data = VALUE_RegOpMode_Tx;
 LoraRegWrite(SpiHandle, LR_RegOpMode,&data,1); //0x8d rx, 0x8b tx
  memset(RxData, 0, RX_BUF_LEN);
 LoraRegRead(SpiHandle, LR_RegOpMode,(char*)RxData,2);	

 LoraRegRead(SpiHandle, LR_RegModemConfig2,(char*)RxData,2);  


 LoraRegRead(SpiHandle,(byte)(LR_RegFifoTxBaseAddr), (char*)RxData,2);	//RegFiFoTxBaseAddr
 //RegFifoAddrPtr 设置指针
 LoraRegWrite(SpiHandle, LR_RegFifoAddrPtr,(uint8*)RxData[0],1);
 
 //memset(RxData, 0, 32);

 //等待TX中断
 while(1)
 {	
	LoraRegRead(SpiHandle, LR_RegIrqFlags, (char*)TxFlgs, 1);
	if((TxFlgs[0] & 0x08) == 0x08)
		break;		
 }
 data = VALUE_LR_RegPADAC_RX;
 LoraRegWrite(SpiHandle, LR_RegPADAC,&data,1);

 return(1);
}

/******************************************************************************
**函数名称：Lora_TxPacket
**函数功能：发射一包数据
**输入参数：无
**输出参数：
******************************************************************************/
byte Lora_TxPacket(SPI_HandleTypeDef *SpiHandle, char *pData, byte iSize)
{

	//FEM_CPS_TX;  //必须调用，切换到发送模式
	RFSW_SET_PIN(SpiHandle,SPI_TX_MODE);	
	
	memset(RxData, 0, 32);
  	LoraRegRead(SpiHandle, (byte)(LR_RegIrqFlags), (char*)RxData,2);		//读取中断Flag，必要是处理		
 	Lora_ClearIrq(SpiHandle);
 
	TxLimtTime = 0;
 	Lora_EntryTx(SpiHandle, pData, iSize);			//??为何不支持连续模式？非要全部重新启动？	
 	return 1;		
}

/**************************************************************
**函数名称: RxPacketQuery
**函数功能：查询是否收到数据
**输入参数：无
**输出参数：无
**************************************************************/
void RxPacketQuery(SPI_HandleTypeDef *SpiHandle)	//查询是否收到数据
{
 byte result;	
 result = 0;
	
 	result = Lora_RxPacket(SpiHandle,(char*)RxData);
 
 if(result)
 	{
 	RxLimtTime = 0;	
 	ChkRxTime = 0;			//有信号情况下，推迟查询状态
 
	//RxActive_F = 1;			//接收中有效
	}
 if(RxLimtTime>=15)			//较长时间没有收到停止
 	{ 
	//RxActive_F = 0;
	}
}




























