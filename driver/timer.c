/**
	********************************************************
	* @file    timer.c
	* @data    2018/08/06
	* @author  vincent
	* @brief   定时器初始化及中断服务函数
  */


/*********************************************************************
*                            头文件                                  *
---------------------------------------------------------------------*/
#include "timer.h"

/*********************************************************************
*                          类型 与 宏定义                            *
---------------------------------------------------------------------*/

/*********************************************************************
*                          全局变量定义                              *
---------------------------------------------------------------------*/
uint32_t u32TimerCnt10ms=0;

/*********************************************************************
*                          局部变量定义                              *
---------------------------------------------------------------------*/
TIM_HandleTypeDef TIM6Handle;


/*********************************************************************
*                            函数实现                                *
**********************************************************************/

/**
	*	@func   TIMER6初始化，基于系统时钟 12MHz
	* @param  None
	* @retval None
	* @brief  10ms一次中断
	*         TIMER6作为延时用
	*/
void Sys10msTimerInit(void)
{
	/* TIMER6初始化 */
	TIM6Handle.Instance = TIM6;
	TIM6Handle.Init.Period = 1200 - 1;
	TIM6Handle.Init.Prescaler = 100-1;
	TIM6Handle.Init.ClockDivision = 0;
	TIM6Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_Base_Init(&TIM6Handle) != HAL_OK)
	{
		/* Initialization Error */
		while(1);
	}	

	/*##-2- Start the TIM Base generation in interrupt mode ####################*/
	/* Start Channel1 */
	if(HAL_TIM_Base_Start_IT(&TIM6Handle) != HAL_OK)
	{
		/* Starting Error */
		while(1);
	}	
}

/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	__TIM6_CLK_ENABLE();            /* 使能TIMER6时钟 */

	HAL_NVIC_SetPriority(TIM6_IRQn, 3, 0); /* 设置优先级 */
	HAL_NVIC_EnableIRQ(TIM6_IRQn); /* Enable the TIMx global Interrupt */
}

/**
  * @brief  TIMER6中断服务函数
  * @param  None
  * @retval None
  */
void TIM6_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TIM6Handle);
}
















