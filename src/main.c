/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComPolling/Src/main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    18-June-2014
  * @brief   This sample code shows how to use STM32L0xx SPI HAL API to transmit 
  *          and receive a data buffer with a communication process based on
  *          Polling transfer. 
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>




#define REQUEST_REPEAT_TIME		(600)   //10 minute
#define CHECK_SPOT_TIME	(600)   // 20 seconds
#define DELAY_MS		100
#define ACK_TIMES		3
#define WAIT_ACK_TIME	(1000*3)  //s
#define HEART_BEAT_TIME (600)

#define UART_INDEX_FIRST		1      
#define UART_INDEX_SECOND		2      
#define UART_INDEX_THIRD		3    


//usb
extern uint8_t Reserve_Data[32];
static volatile uint8_t UsbState=0;
static volatile uint8_t flag=1;
static uint8_t flash_flag=1;
static volatile uint8_t recv_time=0;


//程序崩溃计数
uint32_t ErrorCodeCount;


///* Variable used to get converted value */
//__IO uint32_t uwADCxConvertedValue = 0;

UART_HandleTypeDef		UartHandle1;


MAC_INFO                g_mac_info;
int g_open_the_door=0;
extern int	__ctrl_dev(void);



//用于存储flash读出的LoRa相关寄存器参数
volatile unsigned char Value_LR_RegFrMsb;
volatile unsigned char Value_LR_RegFrMid;
volatile unsigned char Value_LR_RegFrLsb;
volatile unsigned char Value_LR_RegModemConfig1;
volatile unsigned char Value_LR_RegModemConfig2;
volatile unsigned char Value_LR_RegModemConfig3;
volatile unsigned char Value_LR_RegPaConfig;
volatile unsigned char Value_LR_RegPreambleMsb;
volatile unsigned char Value_LR_RegPreambleLsb;

static void Lora_Sleep_handle(SPI_HandleTypeDef *SpiHandle);

/* Uncomment this line to use the board as master, if not it is used as slave */
#define MASTER_BOARD

/* Private variables ---------------------------------------------------------*/
/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;
/* GPIO handler declaration */
GPIO_InitTypeDef GPIO_InitStructure;
/* UART handler declaration */
UART_HandleTypeDef UartHandle1; 
void SPI_Reset(void);


/* USB handle declaration */
USBD_HandleTypeDef USBD_Device;

/* IWDG and TIM handlers declaration */
static IWDG_HandleTypeDef   IwdgHandle;
TIM_HandleTypeDef    Input_Handle;

RCC_ClkInitTypeDef RCC_ClockFreq;
RCC_ClkInitTypeDef RCC_ClkInitStruct;
RCC_OscInitTypeDef RCC_OscInitStruct;

uint16_t tmpCC4[2] = {0, 0};
__IO uint32_t uwLsiFreq = 0;
__IO uint32_t uwCaptureNumber = 0;
__IO uint32_t uwPeriodValue = 0;

        	            
/* Private function prototypes -----------------------------------------------*/
//static void SystemClock_Config(void);
static void SystemClock12_Config(void);
static void SystemClock16_Config(void);

static void SystemClock_USB_Config(void);
//static void SystemClock_close_Config(void);


//static void SystemClockConfig_Stop(void);
static void Error_Handler(void);
static void SystemPower_Config(void);
//static void MX_GPIO_Init(void);
static void MX_GPIO_DeInit(void);
//static void STM32_Standby(void);
//static void STM32_Stop(void);
//static void SPI_Reset(void);
static void System_Init(void);
static void IWDG_Iint(void);
static void Lora_Sleep_handle(SPI_HandleTypeDef *SpiHandle);
void bochiot_Electric_ctrl(char);
uint8_t GetVoltageValue(void);


void bochiot_init(void);


extern byte Lora_RxPacket(SPI_HandleTypeDef *SpiHandle, char* pData);
extern byte Lora_EntryRx(SPI_HandleTypeDef *SpiHandle);
extern void Lora_FEM_CPS_init(void);
extern void Lora_Sleep(SPI_HandleTypeDef *SpiHandle);
extern void Lora_Sleep(SPI_HandleTypeDef *SpiHandle);
extern void Lora_Standby(SPI_HandleTypeDef *SpiHandle);
extern void LoraRegRead(SPI_HandleTypeDef *SpiHandle, byte adr, char* pData, byte read_len);
extern void LoraRegWrite(SPI_HandleTypeDef *SpiHandle, byte, uint8*, int);
extern byte Lora_TxPacket(SPI_HandleTypeDef *SpiHandle, char *pData, byte iSize);
extern void Lora_CadConfig(SPI_HandleTypeDef *SpiHandle);
extern void Lora_ClearIrq(SPI_HandleTypeDef *SpiHandle);
extern void Lora_Config(SPI_HandleTypeDef *SpiHandle);
extern void Lora_Config_Init(SPI_HandleTypeDef *SpiHandle);
HAL_StatusTypeDef FLASH_Write(uint32_t Address, uint32_t *pData, uint32_t Data_len);
uint32_t FLASH_Read(uint32_t Address,uint32_t Read_len,uint32_t *pData, int FlashFlag);



int num = 0;

// ESLAB  RTC DECLARE

extern uint8 bochiot_check_str(uint8 *retPacket,uint8 len);

//添加打印调试信息测试接口
/*
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

struct __FILE 			{ int handle; };   // Add whatever needed 
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) 
{
  if (DEMCR & TRCENA) {
    while (ITM_Port32(0) == 0);
    ITM_Port8(0) = ch;
  }
  return(ch);
}
*/
//调试打印代码结束

void Lora_Reset_On()
{
	
	  HAL_GPIO_WritePin(LORA_RESET_GPIO_PORT,LORA_RESET_PIN, GPIO_PIN_SET); 
}


void SPI_SNN_On()
{

	  HAL_GPIO_WritePin(SPIx_NSS_GPIO_PORT,SPIx_NSS_PIN, GPIO_PIN_SET); 
}

void SPI_SNN_Off()
{

	  HAL_GPIO_WritePin(SPIx_NSS_GPIO_PORT,SPIx_NSS_PIN, GPIO_PIN_RESET); 
}

void LORA_RXLED_On()
{

	  HAL_GPIO_WritePin(LORA_RXLED_GPIO_PORT, LORA_RXLED_PIN, GPIO_PIN_SET); 
}

void LORA_RXLED_Off()
{

	  HAL_GPIO_WritePin(LORA_RXLED_GPIO_PORT, LORA_RXLED_PIN, GPIO_PIN_RESET); 
}
//继电器控制开函数
void LORA_PowerSwitch_On()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	#if Electric_Contorl_New
		GPIO_InitStruct.Pin = LORA_PowerSwitch_PIN_6; 
		GPIO_InitStruct.Speed=GPIO_SPEED_FAST;
		GPIO_InitStruct.Mode= GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP; 
		HAL_GPIO_Init(LORA_PowerSwitch_GPIO_PORT, &GPIO_InitStruct); 
		HAL_GPIO_WritePin(LORA_PowerSwitch_GPIO_PORT, LORA_PowerSwitch_PIN_6, GPIO_PIN_RESET);
	
		GPIO_InitStruct.Pin = LORA_PowerSwitch_PIN_7; 
		GPIO_InitStruct.Speed=GPIO_SPEED_FAST;
		GPIO_InitStruct.Mode= GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP; 
		HAL_GPIO_Init(LORA_PowerSwitch_GPIO_PORT, &GPIO_InitStruct); 	
		HAL_GPIO_WritePin(LORA_PowerSwitch_GPIO_PORT, LORA_PowerSwitch_PIN_7, GPIO_PIN_SET);
	#else	
		HAL_GPIO_WritePin(LORA_PowerSwitch_GPIO_PORT, LORA_PowerSwitch_PIN_6, GPIO_PIN_SET); 
	#endif
}

//继电器控制关函数
void LORA_PowerSwitch_Off()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	#if Electric_Contorl_New
	  GPIO_InitStruct.Pin = LORA_PowerSwitch_PIN_6; 
	  GPIO_InitStruct.Speed=GPIO_SPEED_FAST;
	  GPIO_InitStruct.Mode= GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;  
	  HAL_GPIO_Init(LORA_PowerSwitch_GPIO_PORT, &GPIO_InitStruct); 
	  HAL_GPIO_WritePin(LORA_PowerSwitch_GPIO_PORT, LORA_PowerSwitch_PIN_6, GPIO_PIN_SET); 	
	
	  GPIO_InitStruct.Pin = LORA_PowerSwitch_PIN_7; 
	  GPIO_InitStruct.Speed=GPIO_SPEED_FAST;
	  GPIO_InitStruct.Mode= GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;  
	  HAL_GPIO_Init(LORA_PowerSwitch_GPIO_PORT, &GPIO_InitStruct); 
	  HAL_GPIO_WritePin(LORA_PowerSwitch_GPIO_PORT, LORA_PowerSwitch_PIN_7, GPIO_PIN_RESET);  
	#else 
	   GPIO_InitStruct.Pin = LORA_PowerSwitch_PIN_6; 
	   GPIO_InitStruct.Speed=GPIO_SPEED_FAST;
	   GPIO_InitStruct.Mode= GPIO_MODE_OUTPUT_PP;
	   GPIO_InitStruct.Pull = GPIO_PULLDOWN; 
	   HAL_GPIO_Init(LORA_PowerSwitch_GPIO_PORT, &GPIO_InitStruct);  
	   HAL_GPIO_WritePin(LORA_PowerSwitch_GPIO_PORT, LORA_PowerSwitch_PIN_6, GPIO_PIN_RESET);  
	#endif
}
void SPIx_Write(SPI_HandleTypeDef *hspi, char* Value,   byte size)
{
	HAL_StatusTypeDef status = HAL_OK;

	SPI_SNN_Off();

	status = HAL_SPI_Transmit(hspi, (uint8_t*) Value, size, (uint32_t)0x1000);
	
	
	// Check the communication status 
	if(status != HAL_OK)
	{
		// Re-Initiaize the BUS 
		//SPIx_Error();
		// De-initialize the SPI communication BUS
		HAL_SPI_DeInit(hspi);
	  
		// Re-Initiaize the SPI communication BUS 
		HAL_SPI_Init(hspi);	
	}
}

//static void Lora_Sleep_handle(SPI_HandleTypeDef *SpiHandle)
//{
//	uint8 data;
//	char pdata[32];
//	//进入休眠模式
//	Lora_Sleep(SpiHandle);
//	data = VALUE_LR_RegDIOMAPPING2;
//	LoraRegWrite(SpiHandle,LR_RegDIOMAPPING2,&data,1);
//	Lora_Sleep(SpiHandle);
//}



void System_Init(void)
{
	HAL_Init();
	
	/* Select and configure the system clock */
	if(SelectSystemClock)
	{
		SystemClock16_Config();       //16MHZ
	}
	else
	{
		SystemClock12_Config();       //12MHZ
	}
}



void IWDG_Iint(void)
{	
	
	 /*##-1- Check if the system has resumed from IWDG reset ####################*/
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
	{
		__HAL_RCC_CLEAR_RESET_FLAGS();
	}
	
	/*##-2- Get the LSI frequency: TIM21 is used to measure the LSI frequency ###*/
	//uwLsiFreq = GetLSIFrequency();
	
	/*##-3- Configure & Initialize the IWDG peripheral ######################################*/
	 /* Set counter reload value to obtain 250ms IWDG TimeOut.
		IWDG counter clock Frequency = LsiFreq/32
		Counter Reload Value = 250ms/IWDG counter clock period
							 = 0.25s / (32/LsiFreq)
							 = LsiFreq/(32 * 4)
							 = LsiFreq/128 */
	IwdgHandle.Instance = IWDG;
	IwdgHandle.Init.Prescaler = IWDG_PRESCALER_256;
	IwdgHandle.Init.Reload = 0xC05;
	IwdgHandle.Init.Window = IWDG_WINDOW_DISABLE;
	
	if(HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
	{
	  /* Initialization Error */
	  Error_Handler();
	}
	
	/*##-4- Start the IWDG #####################################################*/ 
	if(HAL_IWDG_Start(&IwdgHandle) != HAL_OK)
	{
	  Error_Handler();
	}
}
void SPI_Reset(void)
{
	SpiHandle.State = HAL_SPI_STATE_RESET;
}

//	  @brief  SPI Read 4 bytes from device	    @retval Read data	 
 uint32_t SPIx_Read(SPI_HandleTypeDef *hspi, char *pData,uint8 read_len)
{
	if(read_len<=0) return  0;
	 
	HAL_StatusTypeDef status = HAL_OK;	
	  
	status = HAL_SPI_Receive(hspi, (uint8_t*) pData, read_len, 1000);
	//status = HAL_SPI_Receive_IT(hspi, (uint8_t*)pData, 2);
	// Check the communication status 
	if(status != HAL_OK)
	{
		/* Re-Initiaize the BUS */
		//SPIx_Error();
		 /* De-initialize the SPI communication BUS */
		HAL_SPI_DeInit(&SpiHandle);
	
		/* Re-Initiaize the SPI communication BUS */
		HAL_SPI_Init(&SpiHandle);
		return 0;
	}
	SPI_SNN_On();
	return 1;
}

void sx1276_burst_read(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t *buf, int len)
{
	SPI_SNN_Off();
	
	HAL_SPI_Transmit(hspi, (uint8_t*) &addr, 1, (uint32_t)0x1000);
    HAL_SPI_Receive(hspi, (uint8_t*) buf, len, 1000);

	SPI_SNN_On();
}

static void	STM32_Stop(void)
{
	SystemPower_Config();
	/* Disable Wakeup Counter */
	HAL_RTCEx_DeactivateWakeUpTimer(get_RTC_Handle());

	/* 此函数用于IO口中断唤醒操作 */
	//	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

	/*## Setting the Wake up time ############################################*/
	/*  RTC Wakeup Interrupt Generation:
	Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI))
	Wakeup Time = Wakeup Time Base * WakeUpCounter 
	= (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI)) * WakeUpCounter
	==> WakeUpCounter = Wakeup Time / Wakeup Time Base

	To configure the wake up timer to 4s the WakeUpCounter is set to 0x1FFF:
	RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16 
	Wakeup Time Base = 16 /(~39.000KHz) = ~0,410 ms
	Wakeup Time = ~4s = 0,410ms  * WakeUpCounter
	==> WakeUpCounter = ~4s/0,410ms = 9750 = 0x2616
	16mhz 810ms=0x680  StopTime
	12mhz 768ms=0x620*/
	HAL_RTCEx_SetWakeUpTimer_IT(get_RTC_Handle(),  StopTime,  RTC_WAKEUPCLOCK_RTCCLK_DIV16);

	/* Enter Stop Mode */
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	/* Configures system clock after wake-up from STOP: enable HSI, PLL and select
	PLL as system clock source (HSI and PLL are disabled automatically in STOP mode) */
	if(SelectSystemClock)
	{
		SystemClock16_Config(); 
	}
	else
	{
		SystemClock12_Config();
	}
}
#if 0
static void SystemClockConfig_Stop(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  
  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);	
	
  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.MSICalibrationValue=0x00;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; 
 
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}
#endif
void open_lock(void)
{
	g_open_the_door = 1;
}

//--------------------------------------------------------------------------------------------
//  Section xxx: 初始化开锁及蜂鸣器引脚
//--------------------------------------------------------------------------------------------
static void lock_and_beep_GPIO_Init(void)
{
	//   lgc0724
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin = GPIO_PIN_0 |GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
//	 lgc0724

}

#if 1
static void MX_GPIO_DeInit(void)
{
	  /* Enable GPIOs clock */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
//  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
//  __GPIOH_CLK_ENABLE();

   /*Configure GPIO pins : PH0 PH1 */
//  GPIO_InitStructure.Pin = GPIO_PIN_0|GPIO_PIN_1;
//  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStructure.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
	
	#if NODE_TYPE
	/* PA组引脚处理 */
//  GPIO_InitStructure.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
//                          |GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
//                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
//                          |GPIO_PIN_15 | GPIO_PIN_5;
//  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStructure.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	#else
	/* PA�����Ŵ��� */
  GPIO_InitStructure.Pin = GPIO_PIN_0|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_8 |GPIO_PIN_9|GPIO_PIN_10
													|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_5;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	#endif
	/* PB组引脚处理 */
	//设置SPI2-NSS为开漏 输出高电平
	GPIO_InitStructure.Pin = GPIO_PIN_12;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIOB->ODR |= (GPIO_ODR_OD0 << 12);//置高
	/*将MOSI、SCLK置低*/
	GPIO_InitStructure.Pin = GPIO_PIN_13 | GPIO_PIN_15;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIOB->ODR &= ~((GPIO_ODR_OD0 << 13) & (GPIO_ODR_OD0 << 15));//置低
	
	/*将miso置低*/
	GPIO_InitStructure.Pin = GPIO_PIN_14;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIOB->ODR &= ~(GPIO_ODR_OD0 << 14);//置低
	
	/* 将DIOx引脚设置为输入高阻态 */
	GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*将RFSW设置为高阻*/
	GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 剩余PB组引脚 */
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | 
													  GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	  /*Configure GPIO pins : PC13 PC14 PC15 PC0 
                           PC1 PC2 PC3 PC4 
                           PC5 PC6 PC7 PC8 
                           PC9 PC10 PC11 PC12 */
//  GPIO_InitStructure.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
//                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
//                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
//                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
//  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStructure.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	
//	/*Configure GPIO pin : PD2 */
//  GPIO_InitStructure.Pin = GPIO_PIN_2;
//  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStructure.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);







  		
	/* Disable GPIOs clock */
  __GPIOA_CLK_DISABLE();
  __GPIOB_CLK_DISABLE();
  __GPIOC_CLK_DISABLE();
  __GPIOD_CLK_DISABLE();
  __GPIOH_CLK_DISABLE();
}
#endif

//jking
/***********************UART****************************************/
//UART 接口操作函数
void UARTx_BochiInit(UART_HandleTypeDef *UartHandle, uint8_t iIndex)
{
	if(iIndex == UART_INDEX_FIRST)
	{
		UartHandle->Instance				 = USART1;
	}
	else if(iIndex == UART_INDEX_SECOND)
	{
		UartHandle->Instance				 = USART2;
	}	
	else
	{
		UartHandle->Instance				 = USART1;
	}
	
	UartHandle->Init.BaudRate   = 115200;
	UartHandle->Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle->Init.StopBits   = UART_STOPBITS_1;
	UartHandle->Init.Parity     = UART_PARITY_NONE;  //奇偶校验
	UartHandle->Init.HwFlowCtl  = UART_HWCONTROL_NONE;//无流控制
	UartHandle->Init.Mode       = UART_MODE_TX_RX;
	if(HAL_UART_DeInit(UartHandle) != HAL_OK)
	{
		//Error_Handler();
	}  
	if(HAL_UART_Init(UartHandle) != HAL_OK)
	{
		//Error_Handler();
	}
}

static void BOCHIUARTx_Error(UART_HandleTypeDef *huart)
{
	HAL_UART_DeInit(huart);
	HAL_UART_Init(huart);
}

void UARTx_Write(UART_HandleTypeDef *huart, char* Value, byte size)
{	
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_UART_Transmit(huart, (uint8_t*)Value, size, (uint32_t)5000);

	if(status != HAL_OK)
	{
		BOCHIUARTx_Error(huart);
	}
}

uint32_t UARTx_Read(UART_HandleTypeDef *huart, char *pData,uint16_t size)
{
	 
	HAL_StatusTypeDef status = HAL_OK;	
//	uint32_t writevalue = 0xFFFFFFFF;
	  
	status = HAL_UART_Receive(huart, (uint8_t *)pData, size, 3000);
	//status = HAL_SPI_Receive_IT(hspi, (uint8_t*)pData, 2);
	// Check the communication status 
	if(status != HAL_OK)
	{
		//Execute user timeout callback 
		BOCHIUARTx_Error(huart);
		return 0;
	}
	return 1;
}



HAL_StatusTypeDef FLASH_Init(void)
{
	HAL_FLASH_Unlock();

	/* Unlock the Options Bytes *************************************************/
	HAL_FLASH_OB_Unlock();

	/* Get pages write protection status ****************************************/
	//HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
	
	return HAL_OK;
}














void bochiot_uart_print(char * buffer)
{
	UARTx_Write(&UartHandle1,buffer,strlen(buffer)+1);
}
void bochiot_send_msg(uint8 *msg, uint8 msg_len)
{	
	
//	Lora_TxPacket(&SpiHandle, (char*)msg, msg_len);	
//	return;
	/*  回包时前道码长度可调短用以增加回报效率 */		
	uint8_t PreambleMsb;
	uint8_t PreambleLsb;	
	
	PreambleLsb = Value_LR_RegPreambleLsb;
	PreambleMsb = Value_LR_RegPreambleMsb;
	Value_LR_RegPreambleLsb = preambleLsb_packetback;
	Value_LR_RegPreambleMsb = preambleMsb_packetback;
	
//	DB_HEX_str("sendmsg:",msg,msg_len);
	Lora_TxPacket(&SpiHandle, (char*)msg, msg_len);	
	//修改PADAC寄存器，降功耗
	byte data;
	data = VALUE_LR_RegPADAC;
	LoraRegWrite(&SpiHandle, LR_RegPADAC,&data,1);
	
	Value_LR_RegPreambleLsb = PreambleLsb;
	Value_LR_RegPreambleMsb = PreambleMsb;
}

void CLOKC_CTROL_On()
{
	  HAL_GPIO_WritePin(CLOCK_GPIO_PORT_8, CLOCK_PIN_8, GPIO_PIN_SET); 
	  HAL_GPIO_WritePin(CLOCK_GPIO_PORT_9, CLOCK_PIN_9, GPIO_PIN_RESET); 
}

void CLOKC_CTROL_OFF()
{

	  HAL_GPIO_WritePin(CLOCK_GPIO_PORT_8, CLOCK_PIN_8, GPIO_PIN_RESET); 
	  HAL_GPIO_WritePin(CLOCK_GPIO_PORT_9, CLOCK_PIN_9, GPIO_PIN_SET); 
}

void CLOKC_CTROL_STANDBY()
{

	  HAL_GPIO_WritePin(CLOCK_GPIO_PORT_8, CLOCK_PIN_8, GPIO_PIN_RESET); 
	  HAL_GPIO_WritePin(CLOCK_GPIO_PORT_9, CLOCK_PIN_9, GPIO_PIN_RESET); 
}

void BEEP_CTROL_ON()
{

	  HAL_GPIO_WritePin(CLOCK_GPIO_PORT_BEEP, CLOCK_PIN_BEEP, GPIO_PIN_SET);
}

void BEEP_CTROL_OFF()
{
		HAL_GPIO_WritePin(CLOCK_GPIO_PORT_BEEP, CLOCK_PIN_BEEP, GPIO_PIN_RESET); 
}


void bochiot_lock_ctrl(void)// lgc 0728
{
	CLOKC_CTROL_OFF();
	BEEP_CTROL_ON(); // lgc 0724
	HAL_Delay(400);	
	CLOKC_CTROL_STANDBY();
	BEEP_CTROL_OFF(); // lgc 0724
	HAL_Delay(5000);  // lgc 0724
	CLOKC_CTROL_On();
	BEEP_CTROL_ON(); // lgc 0728
	HAL_Delay(400);
	BEEP_CTROL_OFF(); // lgc 0728
	CLOKC_CTROL_STANDBY();
}

void bochiot_Electric_ctrl(char ctrol)
{
	if(ctrol)
	{
		LORA_PowerSwitch_On();
	}
	else
	{
		LORA_PowerSwitch_Off();		
	}
}


void bochiot_init(void)
{
	/*##-1- Configure the SPI peripheral #######################################*/
	/* Set the SPI parameters */
	SpiHandle.Instance               = SPIx;

	SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
	SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
	SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
	SpiHandle.Init.CRCPolynomial     = 7;
	SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
	SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	SpiHandle.Init.NSS               = SPI_NSS_SOFT;
	SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
#ifdef MASTER_BOARD
	SpiHandle.Init.Mode = SPI_MODE_MASTER;
#else
	SpiHandle.Init.Mode = SPI_MODE_SLAVE;
#endif /* MASTER_BOARD */

	if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	Lora_FEM_CPS_init();

	Lora_Reset_On();

//	UARTx_BochiInit(&UartHandle1,UART_INDEX_SECOND);
	FLASH_Init();
	lock_and_beep_GPIO_Init();//lgc0731
//	uart_debug_init(bochiot_uart_print);	
}

int RecvConfigMsg(char *recvBuffer)
{
	int len =0;
	return len;
}
/* Private functions ---------------------------------------------------------*/

void Delay_us(uint32_t nus) 
{   
	#if 1
	uint32_t temp; 
	uint32_t uscount;
	uscount = 12*nus;
	SysTick->VAL=0X00;
	SysTick->LOAD = uscount;   
	SysTick->CTRL=0X05;
	do  
	{    
		temp=SysTick->CTRL;
	}while((temp&0x01)&&(!(temp&(1<<16))));
	SysTick->VAL =0X00;
	SysTick->CTRL=0x04;    

	#endif
	#if 0
	nus=nus*1.05;
	nus=nus/1000;
	HAL_Delay(nus);
	#endif
}

/**************usb检测引脚配置*******************/
void USB_Check_PinInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
  __GPIOA_CLK_ENABLE();
	
  GPIO_InitStruct.Pin = GPIO_PIN_usb;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void USB_Check_PinDeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_usb;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	__GPIOA_CLK_DISABLE();
}

void SetLoRaReg(void)//设置LoRa特定寄存器参数
{
	uint8 response1[16];
	int a, b, c;
	g_LoRaReg_info.VALUE_LR_RegFrMsb = Reserve_Data[3];
	g_LoRaReg_info.VALUE_LR_RegFrMid	 = Reserve_Data[12];
	g_LoRaReg_info.VALUE_LR_RegFrLsb =	Reserve_Data[13] ;

	switch(Reserve_Data[4])//带宽
	{
		case 0x01:
			a = 0x70;
			break;
		case 0x02:
			a = 0x80;
			break;
		case 0x03:
			a = 0x90;
			break;
		default:
			break;
	}
	switch(Reserve_Data[6])//纠错编码率
	{
		case 0x01:
			b = 0x02;
			break;
		case 0x02:
			b = 0x04;
			break;
		case 0x03:
			b = 0x06;
			break;
		case 0x04:
			b = 0x08;
			break;
		default:
			break;
	}
	switch(Reserve_Data[7])//报头模式
	{
		case 0x00:
			c = 0x00;
			break;
		case 0x01:
			c = 0x01;
			break;
		default:
			break;
	}
	g_LoRaReg_info.VALUE_LR_RegModemConfig1 = (a|b|c);

	switch(Reserve_Data[5])//扩频因子SF
	{
		case 0x06:
			g_LoRaReg_info.VALUE_LR_RegModemConfig2 = 0x64;
			break;
		case 0x07:
			g_LoRaReg_info.VALUE_LR_RegModemConfig2 = 0x74;
			break;
		case 0x08:
			g_LoRaReg_info.VALUE_LR_RegModemConfig2 = 0x84;
			break;
		case 0x09:
			g_LoRaReg_info.VALUE_LR_RegModemConfig2 = 0x94;
			break;
		case 0x0A:
			g_LoRaReg_info.VALUE_LR_RegModemConfig2 = 0xA4;
			break;
		case 0x0B:
			g_LoRaReg_info.VALUE_LR_RegModemConfig2 = 0xB4;
			break;
		case 0x0C:
			g_LoRaReg_info.VALUE_LR_RegModemConfig2 = 0xC4;
			break;
		default:
			break;
	}

	switch(Reserve_Data[8])//低数据优化
	{
		case 0x00:
			g_LoRaReg_info.VALUE_LR_RegModemConfig3 = 0x00;
			break;
		case 0x01:
			g_LoRaReg_info.VALUE_LR_RegModemConfig3 = 0x08;
			break;
		default:
			break;
	}
	switch(Reserve_Data[9])//发射功率
	{
		case 0x01:
			g_LoRaReg_info.VALUE_LR_RegPaConfig = 0x7E;
			break;
		case 0x02:
			g_LoRaReg_info.VALUE_LR_RegPaConfig = 0x8F;
			break;
		default:
			break;
	}
	g_LoRaReg1_info.VALUE_LR_RegPreambleLsb =  Reserve_Data[11];//前导码低八位
	g_LoRaReg_info.VALUE_LR_RegPreambleMsb =Reserve_Data[10];//前导码
	FLASH_Write(FLASH_LORA_REG_START_ADDR,(uint32_t*)&g_LoRaReg_info,sizeof(g_LoRaReg_info));//配置LoRa寄存器参数1,写入flash保存
	memset(Reserve_Data, 0, sizeof(Reserve_Data));
	memset(response1, 0, sizeof(response1));//发送应答给PC
	memset(&g_LoRaReg_info, 0, sizeof(g_LoRaReg_info));
	FLASH_Write(FLASH_LORA_REG1_START_ADDR,(uint32_t*)&g_LoRaReg1_info,sizeof(g_LoRaReg1_info));//配置LoRa寄存器参数2,写入flash保存
	memset(&g_LoRaReg1_info, 0, sizeof(g_LoRaReg1_info));
	FLASH_Read(FLASH_LORA_REG_START_ADDR,sizeof(g_LoRaReg_info),(uint32_t*)&g_LoRaReg_info,2);//读取寄存器参数1用于判断是否配置成功
	FLASH_Read(FLASH_LORA_REG1_START_ADDR,sizeof(g_LoRaReg1_info),(uint32_t*)&g_LoRaReg1_info,3);//读取寄存器参数2用于判断是否配置成功						
	if((&g_LoRaReg_info!=0)&&(&g_LoRaReg_info!=0))//判断是否配置成功
	{
		response1[3] = 0x01;
		flash_flag = 0;
		Value_LR_RegFrMsb = g_LoRaReg_info.VALUE_LR_RegFrMsb;
		Value_LR_RegFrMid = g_LoRaReg_info.VALUE_LR_RegFrMid;
		Value_LR_RegFrLsb = g_LoRaReg_info.VALUE_LR_RegFrLsb;
		Value_LR_RegModemConfig1 = g_LoRaReg_info.VALUE_LR_RegModemConfig1;
		Value_LR_RegModemConfig2 = g_LoRaReg_info.VALUE_LR_RegModemConfig2;
		Value_LR_RegModemConfig3 = g_LoRaReg_info.VALUE_LR_RegModemConfig3;
		Value_LR_RegPaConfig = g_LoRaReg_info.VALUE_LR_RegPaConfig;
		Value_LR_RegPreambleMsb = g_LoRaReg_info.VALUE_LR_RegPreambleMsb;
		Value_LR_RegPreambleLsb = g_LoRaReg1_info.VALUE_LR_RegPreambleLsb;		
	}
	else
	{
		response1[3] = 0x02;
	}
	response1[0] = 0x32;
	response1[1] = 0x58;
	response1[2] = 0x01;
	response1[15] = 0x58;
	USBD_CUSTOM_HID_SendReport(&USBD_Device, response1, 16);
	if(UartConfigForLora)
		UARTx_Write(&UartHandle1, (char*)response1, 16);
}

void SetDefaultLoRaReg(void)//����LoRa�ض��Ĵ�������
{
	
	//工作频率
	g_LoRaReg_info.VALUE_LR_RegFrMsb = Default_VALUE_LR_RegFrMsb;
	g_LoRaReg_info.VALUE_LR_RegFrMid	 = Default_VALUE_LR_RegFrMid;
	g_LoRaReg_info.VALUE_LR_RegFrLsb =	Default_VALUE_LR_RegFrLsb;
	
	//扩频因子
	g_LoRaReg_info.VALUE_LR_RegModemConfig1 = Default_VALUE_LR_RegModemConfig1;		
	g_LoRaReg_info.VALUE_LR_RegModemConfig2 = Default_VALUE_LR_RegModemConfig2;		
	g_LoRaReg_info.VALUE_LR_RegModemConfig3 = 0x00;
	
	//前导码	
	g_LoRaReg1_info.VALUE_LR_RegPreambleLsb = Default_VALUE_LR_RegPreambleLsb;//前导码低八位
	g_LoRaReg_info.VALUE_LR_RegPreambleMsb =	Default_VALUE_LR_RegPreambleMsb;//前导码
	
	//写入Flash
	FLASH_Write(FLASH_LORA_REG_START_ADDR,(uint32_t*)&g_LoRaReg_info,sizeof(g_LoRaReg_info));//配置LoRa寄存器参数1,写入flash保存

	FLASH_Write(FLASH_LORA_REG1_START_ADDR,(uint32_t*)&g_LoRaReg1_info,sizeof(g_LoRaReg1_info));//配置LoRa寄存器参数2,写入flash保存
	
	
	
		//将接收到地址存储到结构g_endevice_info		
		g_endevice_info.local_addr[0] = 0x02;
		g_endevice_info.local_addr[1] = DevcMAC4;
		g_endevice_info.local_addr[2] = DevcMAC5;
		g_endevice_info.local_addr[3] = DevcMAC6;
		
		NodeSixByteMac[0] = 0x1A;
		NodeSixByteMac[1] = 0xAA;
		NodeSixByteMac[2] = 0x02;
		NodeSixByteMac[3] = DevcMAC4;
		NodeSixByteMac[4] = DevcMAC5;
		NodeSixByteMac[5] = DevcMAC6;	
			
	FLASH_Write(FLASH_MAC_START_ADDR,(uint32_t*)&g_endevice_info,sizeof(g_endevice_info));//将Mac地址写入flash中
	FLASH_Write(FLASH_SetMAC_START_ADDR,(uint32_t*)NodeSixByteMac,sizeof(NodeSixByteMac));//将Mac地址写入flash中
	memset(&g_endevice_info, 0, sizeof(g_endevice_info));
	memset(NodeSixByteMac, 0, sizeof(NodeSixByteMac));
	FLASH_Read(FLASH_MAC_START_ADDR,sizeof(g_endevice_info),(uint32_t*)&g_endevice_info,1);	
	FLASH_Read(FLASH_SetMAC_START_ADDR,sizeof(NodeSixByteMac),(uint32_t*)NodeSixByteMac,4); 

	
		
}


void ReadLoRaReg(void)//读取LoRa特定寄存器参数
{
	uint8 response2[16];
	memset(Reserve_Data, 0, sizeof(Reserve_Data));
	memset(&g_LoRaReg_info, 0, sizeof(g_LoRaReg_info));
	FLASH_Read(FLASH_LORA_REG_START_ADDR,sizeof(g_LoRaReg_info),(uint32_t*)&g_LoRaReg_info,2);	
	memset(&g_LoRaReg1_info, 0, sizeof(g_LoRaReg1_info));
	FLASH_Read(FLASH_LORA_REG1_START_ADDR,sizeof(g_LoRaReg1_info),(uint32_t*)&g_LoRaReg1_info,3);	
	memset(response2, 0, sizeof(response2));//发送应答给PC
	if((&g_LoRaReg_info!=0)&&(&g_LoRaReg1_info!=0))//判断是否配置成功//判断是否读取成功
	{
		response2[3] = 0x01;
	}
	else
	{
		response2[3] = 0x02;
	}
	response2[0] = 0x32;response2[1] = 0x58;response2[2] = 0x02;
	response2[4] = g_LoRaReg_info.VALUE_LR_RegFrMsb;
	response2[5] = g_LoRaReg_info.VALUE_LR_RegFrMid;
	response2[6] = g_LoRaReg_info.VALUE_LR_RegFrLsb;
	response2[7] = g_LoRaReg_info.VALUE_LR_RegModemConfig1;
	response2[8] = g_LoRaReg_info.VALUE_LR_RegModemConfig2;
	response2[9] = g_LoRaReg_info.VALUE_LR_RegModemConfig3;
	response2[10] = g_LoRaReg_info.VALUE_LR_RegPaConfig;
	response2[11] =g_LoRaReg_info.VALUE_LR_RegPreambleMsb;
	response2[12] = g_LoRaReg1_info.VALUE_LR_RegPreambleLsb;
	
	Value_LR_RegFrMsb = g_LoRaReg_info.VALUE_LR_RegFrMsb;
	Value_LR_RegFrMid = g_LoRaReg_info.VALUE_LR_RegFrMid;
	Value_LR_RegFrLsb = g_LoRaReg_info.VALUE_LR_RegFrLsb;
	Value_LR_RegModemConfig1 = g_LoRaReg_info.VALUE_LR_RegModemConfig1;
	Value_LR_RegModemConfig2 = g_LoRaReg_info.VALUE_LR_RegModemConfig2;
	Value_LR_RegModemConfig3 = g_LoRaReg_info.VALUE_LR_RegModemConfig3;
	Value_LR_RegPaConfig = g_LoRaReg_info.VALUE_LR_RegPaConfig;
	Value_LR_RegPreambleMsb = g_LoRaReg_info.VALUE_LR_RegPreambleMsb;
	Value_LR_RegPreambleLsb = g_LoRaReg1_info.VALUE_LR_RegPreambleLsb;	
	response2[15] = 0x58;
	USBD_CUSTOM_HID_SendReport(&USBD_Device, response2, 16);
	if(UartConfigForLora)
		UARTx_Write(&UartHandle1, (char*)response2, 16);
}

void ReadAllRoLaReg(void)//读取所有的LoRa寄存器参数
{
	byte ReadReg[10];
	uint8 response31[16];
	uint8 response32[16];
	uint8 response33[16];
	uint8 response34[16];
	uint8 response35[16];
	memset(Reserve_Data, 0, sizeof(Reserve_Data));
	memset(response31, 0, sizeof(response31));//发送应答给PC数组1
	memset(response32, 0, sizeof(response32));//发送应答给PC数组2
	memset(response33, 0, sizeof(response33));//发送应答给PC数组3
	memset(response34, 0, sizeof(response34));//发送应答给PC数组4
	memset(response35, 0, sizeof(response35));//发送应答给PC数组5
	Lora_Config_Init(&SpiHandle);

	/************************第一组0x06-0x0F*******************************/					
	response31[0] = 0x32;response31[1] = 0x58;response31[2] = 0x03;response31[3] = 0x01;response31[4] = 0x01;
	memset(ReadReg,0,sizeof(ReadReg));	
	for (int i=0;i<10;i++)							 
	{
		LoraRegRead(&SpiHandle, 0x06+i, (char*)ReadReg, 1);	
		response31[5+i]=ReadReg[0];
	}
	response31[15] = 0x58;
	USBD_CUSTOM_HID_SendReport(&USBD_Device, response31, 16);//返回数据给PC
	if(UartConfigForLora)
		UARTx_Write(&UartHandle1, (char*)response31, 16);
	HAL_Delay(200);
	
	/************************第二组0x10-0x19*******************************/										
	response32[0] = 0x32;response32[1] = 0x58;response32[2] = 0x03;response32[3] = 0x01;response32[4] = 0x02;
	response32[5] = 0x10;//0x10寄存器
	memset(ReadReg,0,sizeof(ReadReg));
	LoraRegRead(&SpiHandle, LR_RegIrqFlagsMask, (char*)ReadReg, 1);	
	response32[6] =ReadReg[0];
	LoraRegRead(&SpiHandle, LR_RegIrqFlags, (char*)ReadReg, 1);	
	response32[7] =ReadReg[0];
	response32[8] =VALUE_LR_RegRxNbBytes;
	response32[9] =VALUE_LR_RegRxHeaderCntValueMsb;
	response32[10] =VALUE_LR_RegRxHeaderCntValueLsb;
	response32[11] =VALUE_LR_RegRxPacketCntValueMsb;
	response32[12] =VALUE_LR_RegRxPacketCntValueLsb;
	response32[13] =VALUE_LR_RegModemStat;
	response32[14] =VALUE_LR_RegPktSnrValue;					
	response32[15] = 0x58;	
	
	USBD_CUSTOM_HID_SendReport(&USBD_Device, response32, 16);//返回数据给PC
	if(UartConfigForLora)
		UARTx_Write(&UartHandle1, (char*)response32, 16);
	HAL_Delay(200);
	
	/************************第三组0x1A-0x23*******************************/								
	response33[0] = 0x32;response33[1] = 0x58;response33[2] = 0x03;response33[3] = 0x01;response33[4] = 0x03;//第三组0x15-0x1E
	response33[5] = VALUE_LR_RegPktRssiValue;
	response33[6] = VALUE_LR_RegRssiValue;
	response33[7] = VALUE_LR_RegHopChannel;
	memset(ReadReg,0,sizeof(ReadReg));
	for (int i=0;i<7;i++)							 
	{
		LoraRegRead(&SpiHandle, 0x1D+i, (char*)ReadReg, 1);	
		response33[8+i]=ReadReg[0];
	}
	response33[15] = 0x58;	
	USBD_CUSTOM_HID_SendReport(&USBD_Device, response33, 16);//返回数据给PC
	if(UartConfigForLora)
		UARTx_Write(&UartHandle1, (char*)response33, 16);
	HAL_Delay(200);
	
	/************************第四组0x24-0x26,0x40-0x42,,0x4B,0x4D,0x5B,0x61*******************************/				
	response34[0] = 0x32;response34[1] = 0x58;response34[2] = 0x03;response34[3] = 0x01;response34[4] = 0x04;//第四组0x1F-0x26,0x40,0x41
	memset(ReadReg,0,sizeof(ReadReg));
	for (int i=0;i<3;i++)							 
	{
		LoraRegRead(&SpiHandle, 0x24+i, (char*)ReadReg, 1);		
		response34[5+i]=ReadReg[0];
	}
	response34[6]=VALUE_LR_RegFifoRxByteAddr	;
	for (int i=0;i<3;i++)							 
	{
		LoraRegRead(&SpiHandle, 0x40+i, (char*)ReadReg, 1);		
		response34[8+i]=ReadReg[0];
	}
	memset(ReadReg,0,sizeof(ReadReg));
	LoraRegRead(&SpiHandle, 0x4B, (char*)ReadReg, 1);		
	response34[11] = ReadReg[0];	
	memset(ReadReg,0,sizeof(ReadReg));
	LoraRegRead(&SpiHandle, 0x4D, (char*)ReadReg, 1);		
	response34[12] = ReadReg[0];	
	memset(ReadReg,0,sizeof(ReadReg));
	LoraRegRead(&SpiHandle, 0x5B, (char*)ReadReg, 1);		
	response34[13] = ReadReg[0];	
	memset(ReadReg,0,sizeof(ReadReg));
	LoraRegRead(&SpiHandle, 0x61, (char*)ReadReg, 1);		
	response34[14] = ReadReg[0];	
	response34[15] = 0x58;
	USBD_CUSTOM_HID_SendReport(&USBD_Device, response34, 16);//返回数据给PC
	if(UartConfigForLora)
		UARTx_Write(&UartHandle1, (char*)response34, 16);
	HAL_Delay(200);
	
	/************************第五组0x62,0x63,0x64,0x70*******************************/									
	response35[0] = 0x32;response35[1] = 0x58;response35[2] = 0x03;response35[3] = 0x01;response35[4] = 0x05;//第五组0x42,0x4B,0x4D,0x5B,0x61-0x64,0x70					
	memset(ReadReg,0,sizeof(ReadReg));
	for (int i=0;i<3;i++)							 
	{
		LoraRegRead(&SpiHandle, 0x62+i, (char*)ReadReg, 1);		
		response35[5+i]=ReadReg[0];
	}
	memset(ReadReg,0,sizeof(ReadReg));
	LoraRegRead(&SpiHandle, 0x70, (char*)ReadReg, 1);		
	response35[8] = ReadReg[0];	
	response35[15] = 0x58;
	USBD_CUSTOM_HID_SendReport(&USBD_Device, response35, 16);//返回数据给PC
	if(UartConfigForLora)
			UARTx_Write(&UartHandle1, (char*)response35, 16);
	HAL_Delay(200);
}


void SetMacAddress(void)//设置网关和节点Mac地址
{
	uint8 response6[16];
	byte MacAddr[12]; 		
	memset(NodeSixByteMac, 0, sizeof(NodeSixByteMac));
	for(int x=0;x<12;x++)
	{
		MacAddr[x] = Reserve_Data[3+x];
	}	
	memset(Reserve_Data, 0, sizeof(Reserve_Data));
	memset(response6, 0, sizeof(response6));//发送应答给PC
	for (int x=0;x<6;x++)
	{
		NodeSixByteMac[x]=MacAddr[x];//将接收到地址存储到结构体g_mac_info中
		g_endevice_info.local_addr[x]=MacAddr[x+2];//将接收到地址存储到结构g_endevice_info
		g_endevice_info.server_addr[x]=MacAddr[x+6];
	}
	
	FLASH_Write(FLASH_MAC_START_ADDR,(uint32_t*)&g_endevice_info,sizeof(g_endevice_info));//将Mac地址写入flash中
	FLASH_Write(FLASH_SetMAC_START_ADDR,(uint32_t*)NodeSixByteMac,sizeof(NodeSixByteMac));//将Mac地址写入flash中
	memset(&g_endevice_info, 0, sizeof(g_endevice_info));
	memset(NodeSixByteMac, 0, sizeof(NodeSixByteMac));
	FLASH_Read(FLASH_MAC_START_ADDR,sizeof(g_endevice_info),(uint32_t*)&g_endevice_info,1);	
	FLASH_Read(FLASH_SetMAC_START_ADDR,sizeof(NodeSixByteMac),(uint32_t*)NodeSixByteMac,4); 

	if(g_endevice_info.local_addr[0]!=0)//判断是否配置成功
	{response6[3] = 0x01;}
	else{response6[3] = 0x02;}
	response6[0] = 0x32;response6[1] = 0x58;response6[2] = 0x06;response6[15] = 0x58;
	USBD_CUSTOM_HID_SendReport(&USBD_Device, response6, 16);//发送应答数组给PC
	if(UartConfigForLora)
		UARTx_Write(&UartHandle1, (char*)response6, 16);
}


//从FLASH中读取节点mac以及网关mac
void GetMAC(void)
{
	memset((char*)&g_endevice_info,0,sizeof(g_endevice_info));
	FLASH_Read(FLASH_MAC_START_ADDR,sizeof(g_endevice_info),(uint32_t*)&g_endevice_info,1);
	memset((char*)NodeSixByteMac, 0, sizeof(NodeSixByteMac));
	FLASH_Read(FLASH_SetMAC_START_ADDR, sizeof(NodeSixByteMac),(uint32_t*)NodeSixByteMac,4);
}


void ReadLoRaRegFromFlash(void)//从flash中读取特定LoRa寄存器参数
{
#if 0
	memset(&g_LoRaReg_info, 0, sizeof(g_LoRaReg_info));
	FLASH_Read(FLASH_LORA_REG_START_ADDR,sizeof(g_LoRaReg_info),(uint32_t*)&g_LoRaReg_info,2);
	memset(&g_LoRaReg1_info, 0, sizeof(g_LoRaReg1_info));
	FLASH_Read(FLASH_LORA_REG1_START_ADDR,sizeof(g_LoRaReg1_info),(uint32_t*)&g_LoRaReg1_info,3);
	Value_LR_RegFrMsb = g_LoRaReg_info.VALUE_LR_RegFrMsb;
	Value_LR_RegFrMid = g_LoRaReg_info.VALUE_LR_RegFrMid;
	Value_LR_RegFrLsb = g_LoRaReg_info.VALUE_LR_RegFrLsb;
	Value_LR_RegModemConfig1 = g_LoRaReg_info.VALUE_LR_RegModemConfig1;
	Value_LR_RegModemConfig2 = g_LoRaReg_info.VALUE_LR_RegModemConfig2;
	Value_LR_RegModemConfig3 = g_LoRaReg_info.VALUE_LR_RegModemConfig3;
	Value_LR_RegPaConfig = g_LoRaReg_info.VALUE_LR_RegPaConfig;
	Value_LR_RegPreambleMsb = g_LoRaReg_info.VALUE_LR_RegPreambleMsb;
	Value_LR_RegPreambleLsb = g_LoRaReg1_info.VALUE_LR_RegPreambleLsb;	
#endif
	
#if 1
	Value_LR_RegFrMsb = 0x6c;
	Value_LR_RegFrMid = 0x80;
	Value_LR_RegFrLsb = 0x00;
	Value_LR_RegModemConfig1 = 0x72;
	Value_LR_RegModemConfig2 = 0x94;
	Value_LR_RegModemConfig3 = 0x00;
	Value_LR_RegPaConfig = 0x8f;
	Value_LR_RegPreambleMsb = 0x01;
	Value_LR_RegPreambleLsb = 0x6e;
#endif
}

void Test_Rx(void)//接收数据（测试用）
{
	byte RxIrqFlge[32]; 
	char  Recv_msg[128];
	uint8 Recv_len = 0;							
	uint8 responseRx[16];
	memset(responseRx,0,16);
	memset(Reserve_Data,0,16);
	int b;									 
	uint32_t tickstart = 0;
	tickstart = HAL_GetTick();
	while(HAL_GetTick() - tickstart<5000)
	{
		responseRx[0] = 0x32;responseRx[1] = 0x58;responseRx[2] = 0x05;responseRx[15] = 0x58;
		bochiot_init();
		Lora_Config(&SpiHandle);
		Lora_CadConfig(&SpiHandle);
		HAL_Delay(8);
		LoraRegRead(&SpiHandle, LR_RegIrqFlags, (char*)RxIrqFlge, 1);
		if(((RxIrqFlge[0] & 0x05) == 0x05))
		{
			memset(Recv_msg,0,128);			
			Lora_EntryRx(&SpiHandle);
			Recv_len = Lora_RxPacket(&SpiHandle, Recv_msg);
		}
		if (Recv_len  >0)
		{
			b = Recv_len;
			for(int d =0;d<b;d++)
			{
				responseRx[4+d] = Recv_msg[1+d];
			}
			responseRx[3] = 0x01;
			responseRx[15] = 0x58;
			USBD_CUSTOM_HID_SendReport(&USBD_Device, responseRx, 16);//发送应答数组给PC
			if(UartConfigForLora)
				UARTx_Write(&UartHandle1, (char*)responseRx, 16);
			Recv_len = 0;
			memset(responseRx,0,16);
			tickstart = HAL_GetTick();
		}
		memset(Recv_msg,0,sizeof(Recv_msg));
	}
	
	responseRx[0] = 0x32;responseRx[1] = 0x58;responseRx[2] = 0x05;responseRx[15] = 0x58;responseRx[3] = 0x02;
	USBD_CUSTOM_HID_SendReport(&USBD_Device, responseRx, 16);//发送应答数组给PC
	if(UartConfigForLora)
		UARTx_Write(&UartHandle1, (char*)responseRx, 16);
}


void UsbConfigFromPc(void)//上位机利用USB对LoRa进行相关配置
{
	char  Tx_msg[128];
	uint8 responseTx[16];
	uint8 Length;
	USB_Check_PinInit();

	/*****************检测是否有USB接入***********************/
	if((GPIOA->IDR & GPIO_PIN_usb) == GPIO_PIN_usb)
	{
		SystemClock_USB_Config();
		bochiot_init();
		USBD_Init(&USBD_Device, &HID_Desc, 0);
		//HAL_Delay(2000);
		/* Register the HID class */
		USBD_RegisterClass(&USBD_Device, &USBD_CUSTOM_HID);
		/* Start Device Process */
		USBD_Start(&USBD_Device);
		HAL_Delay(2000);
		if(UartConfigForLora)
		{
			UARTx_BochiInit(&UartHandle1,UART_INDEX_SECOND);
//			UARTx_Write(&UartHandle1,"uart test success!", sizeof("uart test success!"));
		}

		while(1)
		{
			if(UartConfigForLora)
				UARTx_Read(&UartHandle1, (char*)Reserve_Data, 16);
			if(Reserve_Data[0]== 0x32&&Reserve_Data[1]== 0x58)//验证校验位
			{
				if(Reserve_Data[2]== 0x01)//配置LoRa寄存器
				{
					SetLoRaReg();
					bochiot_init();
				}		
				else if(Reserve_Data[2]== 0x02)//读取特定寄存器
				{
					ReadLoRaReg();
				}

				else if(Reserve_Data[2]== 0x03)//读取所有寄存器
				{
					if( flash_flag == 1)
					{
						ReadLoRaRegFromFlash();
						bochiot_init();
						flash_flag = 0;
					}
					ReadAllRoLaReg();
				}
				else if(Reserve_Data[2]== 0x06)//配置网关和节点Mac地址
				{
					SetMacAddress();
				}
				else if(Reserve_Data[2]== 0x04)//发送数据，数据由用户输入，小于11字节，详见通信协议
				{
					if( flash_flag == 1)
					{
						ReadLoRaRegFromFlash();
						bochiot_init();
						flash_flag = 0;
					}
					memset(responseTx,0,16);
					responseTx[0] = 0x32;responseTx[1] = 0x58;responseTx[2] = 0x04;responseTx[15] = 0x58;
					memset(Tx_msg,0,128);

					Length = Reserve_Data[4];
					recv_time= Reserve_Data[3];
					Tx_msg[0]= Length;
					for (int c=0;c<Length;c++)
					{
					Tx_msg[1+c] = Reserve_Data[5+c];
					}
					memset(Reserve_Data,0,16);
				}
				else if(Reserve_Data[2]== 0x05)//接收指定Mac地址LoRa模块的数据，详见通信协议
				{			
					if( flash_flag == 1)
					{
						ReadLoRaRegFromFlash();
						bochiot_init();
						flash_flag = 0;
					}
					Test_Rx();	
				}
				else if(Reserve_Data[2]== 0x07)//停止发送
				{
					recv_time = 0;
					memset(Reserve_Data,0,16);
				}
				else if(Reserve_Data[2]== 0x08)//读取mac地址
				{
					uint8 response6[16];
					memset(Reserve_Data,0,16);
					response6[0] = 0x32;response6[1] = 0x58;response6[2] = 0x08;response6[15] = 0x58;
					
					//	FLASH_Write(FLASH_SetMAC_START_ADDR,(uint32_t*)NodeSixByteMac,sizeof(NodeSixByteMac));//将Mac地址写入flash中
					memset(&g_endevice_info, 0, sizeof(g_endevice_info));
					FLASH_Read(FLASH_MAC_START_ADDR,sizeof(g_endevice_info),(uint32_t*)&g_endevice_info,1);	
					memset(NodeSixByteMac, 0, sizeof(NodeSixByteMac));
					FLASH_Read(FLASH_SetMAC_START_ADDR,sizeof(NodeSixByteMac),(uint32_t*)NodeSixByteMac,4);	
					for(int i =0;i<6;i++)
					{
						response6[3+i] = NodeSixByteMac[i];
						response6[9+i] = g_endevice_info.server_addr[i];
					}
					USBD_CUSTOM_HID_SendReport(&USBD_Device, response6, 16);//发送应答数组给PC
					if(UartConfigForLora)
						UARTx_Write(&UartHandle1, (char*)response6, 16);
				}
				memset(Reserve_Data, 0, 16);
			}						
			if(recv_time>0)
			{
				Lora_TxPacket(&SpiHandle, Tx_msg,Length+1);
				responseTx[3] =   responseTx[3]+1;
				USBD_CUSTOM_HID_SendReport(&USBD_Device, responseTx, 16);//发送应答数组给PC
				if(UartConfigForLora)
					UARTx_Write(&UartHandle1, (char*)responseTx, 16);
				recv_time--;
			}			
		}
	}	
	USB_Check_PinDeInit();
}



//--------------------------------------------------------------------------------------------
//  Section xxx: 接收lora数据报并解析
//--------------------------------------------------------------------------------------------

int LORA_RX(void)
{
	static volatile int	 ret = -1;
	static byte RxIrqFlg[32]; 		
	static volatile uint32_t len=0;

	static uint8 recv_len = 0;
	static volatile uint8 wait_ack_flag=0,wait_ack_time=0,wait_count_time=0; // if 1, stm32 should wait for the ack, not enter sleep mode
	static volatile int  check_count_time = 0,heart_beat_time=0;

	static char  recv_msg[128];
		//开启CAD中断
	Lora_CadConfig(&SpiHandle);
	HAL_Delay(8);
	LoraRegRead(&SpiHandle, LR_RegIrqFlags, (char*)RxIrqFlg, 1);
	//CadDetected & CadDone  RxData[0] == 0x05
	if(((RxIrqFlg[0] & 0x05) == 0x05))
	{
		memset(recv_msg,0,128);			
		Lora_EntryRx(&SpiHandle);
//			LORA_RXLED_On();
		recv_len = Lora_RxPacket(&SpiHandle, recv_msg);
		
			//lora在此处进行sleep模式
		Lora_Sleep(&SpiHandle);
		
		//if(recv_len>0 && recv_msg[0]==0xb0)
		if(recv_len>0)
		{
			//recv the msg  
			//UARTx_Write(&UartHandle1, recv_msg, recv_len);
			ret = bochiot_anaylyze_frame((uint8*)recv_msg,recv_len);
			wait_ack_flag = 0;
		}
		//LORA_RXLED_Off();	
		wait_count_time++;
		check_count_time++;
		heart_beat_time++;
		//DB_int(check_count_time,"checktime=","\r\n");
		//HAL_Delay(DELAY_MS);
		return ret;
	}
	return 0;
}





//lgc0731 
//--------------------------------------------------------------------------------------------
//  Section xxx: 外部中断处理函数，刷卡或按键时进入
//--------------------------------------------------------------------------------------------
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t type = 0x00;
	uint8_t recevmesg_lock[14];

	if(GPIO_Pin == KEY_BUTTON_PIN)
	{	

		if(NTM_ReadRandom(recevmesg_lock)==1){//lgc 0724 read the message of card		 
			type = recevmesg_lock[0]; 
			if(type == RSP_I2C_MSG_CARD_A ||type == RSP_I2C_MSG_CARD_B ){					
				swipe_card_handling(recevmesg_lock);											
			}	
			
			
			
		}		
	} 
}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

	
int main(void)
{	
	//usb配置寄存器
	System_Init();//	UsbConfigFromPc();
	SystemPower_Config();// Configure the system Power 
	bochiot_init();	
	bochiot_protocol_init(bochiot_send_msg);
	//设置LoRa特定寄存器参数
	SetDefaultLoRaReg(); 
	HAL_Delay(5000);
	ReadLoRaRegFromFlash();//从flash中读取LoRa特定寄存器参数
	GetMAC();//从FLASH中读取节点mac以及网关mac
//	Sys10msTimerInit();

	#if voltagebutton
	memset(&g_voltage_value, 0, sizeof(g_voltage_value));
	FLASH_Read(FLASH_Get_Voltage_START_ADDR, sizeof(g_voltage_value), (uint32_t*)&g_voltage_value,5);
	g_voltage_value.VoltageValue[6] = g_voltage_value.VoltageValue[6]+1;
	FLASH_Write(FLASH_Get_Voltage_START_ADDR,(uint32_t*)&g_voltage_value,sizeof(g_voltage_value));
	#endif
	
	IWDG_Iint();
	bochiot_init();
	Lora_Config(&SpiHandle);
	ReadLoRaRegFromFlash();//从flash中读取LoRa特定寄存器参数
	ekey_flash_init();	

	uint8 i=0;
	while(1){
		i++;
		if(i >= 10 || time_request() == 0x08)//
			break;	
	}	
	BEEP_CTROL_ON(); // lgc 0728
	HAL_Delay(40);
	BEEP_CTROL_OFF(); // lgc 0728
	NTM_Init();//IIC初始化
	while(1)
	{		
		if(g_open_the_door == 1)  
		{
			__ctrl_dev();//lgc 0723
			g_open_the_door = 0;
		}		
		log_tx();//发送日志
		LORA_RX();//lora 接收
  			
	    GetVoltageValue();//     电量检测程序 

		HAL_Delay(1500);
#if 0//NODE_TYPE    
		/* Lora进入睡眠模式 */
		Lora_Sleep_handle(&SpiHandle);		
		/* GPIO处理 */
		MX_GPIO_DeInit();  // lgc 0723
		/* SPI复位 */
		SPI_Reset();		
		/* STM32进入停止模式 */ 	
//		LORA_RXLED_Off();
		STM32_Stop();
//		LORA_RXLED_On();
		bochiot_init();

#endif
		//喂狗
		if(HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK)
		{
			/* Refresh Error */
			Error_Handler();
		}		
	}		
}


/*12mhz*/
static void SystemClock12_Config(void)
{ 
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to coduct datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_4;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow:
  *  
  *            HSI48 used as USB clock source (USE_USB_CLKSOURCE_CRSHSI48 defined in main.h)
  *              - System Clock source            = HSI
  *              - HSI Frequency(Hz)              = 16000000
  *              - SYSCLK(Hz)                     = 16000000
  *              - HCLK(Hz)                       = 16000000
  *              - AHB Prescaler                  = 1
  *              - APB1 Prescaler                 = 1
  *              - APB2 Prescaler                 = 1
  *              - Flash Latency(WS)              = 0
  *              - Main regulator output voltage  = Scale1 mode
  *
  *            PLL(HSE) used as USB clock source (USE_USB_CLKSOURCE_PLL defined in main.h)
  *              - System Clock source            = PLL (HSE)
  *              - HSE Frequency(Hz)              = 8000000
  *              - SYSCLK(Hz)                     = 32000000
  *              - HCLK(Hz)                       = 32000000
  *              - AHB Prescaler                  = 1
  *              - APB1 Prescaler                 = 1
  *              - APB2 Prescaler                 = 1
  *              - PLL_MUL                        = 12
  *              - PLL_DIV                        = 3
  *              - Flash Latency(WS)              = 1
  *              - Main regulator output voltage  = Scale1 mode
  *
  * @param  None
  * @retval None
  */
static void SystemClock_USB_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
#if defined (USE_USB_CLKSOURCE_CRSHSI48)
  static RCC_CRSInitTypeDef RCC_CRSInitStruct;
#endif

#if defined (USE_USB_CLKSOURCE_CRSHSI48)
 
  /* Enable HSI Oscillator to be used as System clock source
     Enable HSI48 Oscillator to be used as USB clock source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
 //  RCC_OscInitStruct.HSI48State = RCC_HSI48_OFF;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_4;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  HAL_RCC_OscConfig(&RCC_OscInitStruct); 
 
  /* Select HSI48 as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select HSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clock dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
    
  /*Configure the clock recovery system (CRS)**********************************/
 #if 1 
  /*Enable CRS Clock*/
  __CRS_CLK_ENABLE(); 
  
  /* Default Synchro Signal division factor (not divided) */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;  
  /* Set the SYNCSRC[1:0] bits according to CRS_Source value */
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;  
  /* HSI48 is synchronized with USB SOF at 1KHz rate */
  RCC_CRSInitStruct.ReloadValue =  __HAL_RCC_CRS_CALCULATE_RELOADVALUE(48000000, 1000);
  RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;  
  /* Set the TRIM[5:0] to the default value*/
  RCC_CRSInitStruct.HSI48CalibrationValue = 0x20;   
  /* Start automatic synchronization */ 
  HAL_RCCEx_CRSConfig (&RCC_CRSInitStruct);
#endif  
#elif defined (USE_USB_CLKSOURCE_PLL)
     
  /* Enable HSE Oscillator and activate PLL with HSE as source
     PLLCLK = (8 * 12) / 3) = 32 MHz */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct); 
  
  /*Select PLL 48 MHz output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clock dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
  
#endif /*USE_USB_CLKSOURCE_CRSHSI48*/
  
  /* Enable Power Controller clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
  clocked below the maximum system frequency, to update the voltage scaling value 
  regarding system frequency refer to product datasheet. */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
}




/* 16MHZ */
void SystemClock16_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  __SYSCFG_CLK_ENABLE();

}

static void Lora_Sleep_handle(SPI_HandleTypeDef *SpiHandle)
{
	uint8 data;
	//进入休眠模式
	Lora_Sleep(SpiHandle);
	data = VALUE_LR_RegDIOMAPPING2;
	LoraRegWrite(SpiHandle,LR_RegDIOMAPPING2,&data,1);
	Lora_Sleep(SpiHandle);
}

/////JinPeng
/**
  * @brief  System Power Configuration
  *         The system Power is configured as follow :
  *            + RTC Clocked by LSI
  *            + VREFINT OFF, with fast wakeup enabled
  *            + No IWDG
  *            + Automatic Wakeup using RTC clocked by LSI (after ~4s)
  * @param  None
  * @retval None
  */
#if 1
static void SystemPower_Config(void)
{
	/* Enable Ultra low power mode */
	HAL_PWREx_EnableUltraLowPower();

	/* Enable the fast wake up from Ultra low power mode */
	HAL_PWREx_EnableFastWakeUp();

	/* Select MSI as system clock source after Wake Up from Stop mode */
	__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_StopWakeUpClock_MSI);
	
	RTC_Handle_init();

}
#endif
/**
  * @brief  SPI error callbacks
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  /* Infinite loop */
  while(1)
  {
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Infinite loop */
  while(1)
  {
  }
}

/**
  * @brief  RTC Wake Up callback
  * @param  None
  * @retval None
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

/**
  * @brief SYSTICK callback
  * @param None
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
  HAL_IncTick();
}

/**
  * @brief  Input Capture callback in non blocking mode 
  * @param  htim : TIM IC handle
  * @retval None
*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{    
  /* Get the Input Capture value */
  tmpCC4[uwCaptureNumber++] = HAL_TIM_ReadCapturedValue(&Input_Handle, TIM_CHANNEL_1);
  
  if (uwCaptureNumber >= 2)
  {
    /* Compute the period length */
    uwPeriodValue = (uint16_t)(0xFFFF - tmpCC4[0] + tmpCC4[1] + 1);
    
    /* Frequency computation */ 
    uwLsiFreq = (uint32_t) SystemCoreClock / uwPeriodValue;
    uwLsiFreq *= 8;
  }
}

