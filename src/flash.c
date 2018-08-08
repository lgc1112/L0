/**
  ********************************************************
  * @file    flash.c
  * @data    2018/08/05
  * @author  vincent
  * @brief   BOCHIOT SMART LOCK
  */

/*********************************************************************
*                            头文件                                  *
---------------------------------------------------------------------*/
#include "flash.h"

/*********************************************************************
*                          类型 与 宏定义                            *
---------------------------------------------------------------------*/


/*********************************************************************
*                          全局变量定义                              *
---------------------------------------------------------------------*/
uint32_t PageError = 0;

ENDEVICE_INFO_t           g_endevice_info;
LORAREG_INFO_t            g_LoRaReg_info;
LORAREG1_INFO_t          g_LoRaReg1_info;






//用于存放节点6字节mac地址
__align(4) uint8_t NodeSixByteMac[8];
__align(4) BatteryData  g_voltage_value;


/*********************************************************************
*                          局部变量定义                              *
---------------------------------------------------------------------*/
static FLASH_EraseInitTypeDef EraseInitStruct;


/*********************************************************************
*                            函数实现                                *
**********************************************************************/

/**
	* @func   擦除flash
	* @param  None
	* @retval None
	* @brief  None
	*/
static HAL_StatusTypeDef FLASH_Erase(uint32_t TypeErase,uint32_t PageAddress,uint32_t NbPages)
{
	EraseInitStruct.TypeErase = TYPEERASE_PAGEERASE;
	EraseInitStruct.Page = PageAddress;
	EraseInitStruct.NbPages = NbPages;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}	

/**
	* @func   flash写入一个字操作
	* @param  None
	* @retval None
	* @brief  None
	*/
static HAL_StatusTypeDef FLASH_Write_Word(uint32_t Address, uint32_t Data)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t TypeProgram = TYPEPROGRAM_WORD;

	status = HAL_FLASH_Program(TypeProgram, Address,Data);
	return status;
}

/**
	* @func   从指定的地址向flash写入指定的长度
	* @param  写入地址(0-2047)   数据指针  数据长度
	* @retval None
	* @brief  注意！！写入的数据类型必须是 1个字(4个字节)
	*/
HAL_StatusTypeDef FLASH_Write(uint32_t Address, uint32_t *pData, uint32_t Data_len)
{
	uint32_t count= 0;
	FLASH_Erase(0, Address, 4);  //counter 

	/* 由于*pData为uint32_t类型的 所以pData++为四字节*/
	count = Data_len / 4;
	if(Data_len % 4 != 0){
		count++;
	}
	Data_len = count;
	
	for(; Data_len > 0; Data_len--){
		if(FLASH_Write_Word(Address, *pData) != HAL_OK){
			return HAL_ERROR;
		}
		pData++;
		Address += 4;
		
	}
	return HAL_OK;
}

/**
	* @func   flash读取一个字操作
	* @param  None
	* @retval None
	* @brief  None
	*/
static void FLASH_Read_Word(uint32_t Address, uint32_t *pData)
{
	*pData = (*(__IO uint32_t*) Address);
}

/**
	* @func   flash读操作
	* @param  None
	* @retval None
	* @brief  None
	*/
uint32_t FLASH_Read(uint32_t Address, uint32_t Read_len, uint32_t *pData, int FlashFlag)
{
	uint32_t len = 0;
	uint32_t addr = 0;
	uint32_t count = 0;
	
	if(FlashFlag == 1){
		addr = FLASH_MAC_END_ADDR;
	}else if(FlashFlag == 2){
		addr = FLASH_LORA_REG_END_ADDR;
	}else if(FlashFlag == 3){
	addr = FLASH_LORA_REG1_END_ADDR;
	}else  if(FlashFlag == 4){
		addr = FLASH_SetMAC_END_ADDR;
	}else  if(FlashFlag == 5){
		addr = FLASH_Get_Voltage_END_ADDR;
	}
	
	/* 由于*pData为uint32_t类型的 所以pData++为四字节*/
	count = Read_len / 4;
	if(Read_len % 4 != 0){
		count++;
	}
	
	while (Address < addr)
	{
		FLASH_Read_Word(Address, pData);
		pData++;
		Address += 4;

		len ++;
		if(len >= count)
			break;
	}
	return len;
}

/**
	* @func   从flash中读取日志、密码数
	* @param  None
	* @retval None
	* @brief  None
	*/
uint32_t ekey_flash_read(uint32_t Address, uint32_t Read_len, uint32_t *pData, int FlashFlag)
{
	uint32_t len = 0;
	uint32_t addr = 0;
	uint32_t count = 0;
	
	if(FlashFlag == 1){
		addr = FLASH_LOG_END_ADDR;
	}else if(FlashFlag == 2){
		addr = FLASH_QUEUE_END_ADDR;
	}else if(FlashFlag == 3){
	    addr = FLASH_PASSWORD_END_ADDR;
	}
	
	/* 由于*pData为uint32_t类型的 所以pData++为四字节*/
	count = Read_len / 4;
	if(Read_len % 4 != 0){
		count++;
	}
	
	while (Address < addr)
	{
		FLASH_Read_Word(Address, pData);
		pData++;
		Address += 4;

		len ++;
		if(len >= count)
			break;
	}
	
	return len;
}




//void flash_test(uint32_t ReadAddr, uint8_t *pBuffer, uint32_t Length)
//{
//	uint32_t /*addr=0x0800F080, */i=0, value;
//	
//	HAL_FLASH_Unlock();
//	FLASH_Erase(0 ,0x0800F000, 32);
//	
//	for(i=0; i<Length; i+=4){
//			
//		value =  (uint32_t)pBuffer[3]<<24;
//		value += (uint32_t)pBuffer[2]<<16;
//		value += (uint32_t)pBuffer[1]<<8;
//		value += (uint32_t)pBuffer[0];
//		
//		HAL_FLASH_Program(TYPEPROGRAM_WORD, ReadAddr, value);
//		pBuffer += 4;
//		ReadAddr += 4;
//		value = 0;
//	}
//	
//	HAL_FLASH_Lock();
//}


 





