#ifndef __flash_H_
#define __flash_H_
#include "stm32l0xx.h"
#include "bochiot_protocol.h"

#define MAC_LEN			4
#define MAC_GW_LEN		6

/* MAC地址存储区域 */
#define ADDR_FLASH_MAC_PAGE1 ((uint32_t)0x0800D000) /* Base @ of Page 240, 256 bytes */
#define ADDR_FLASH_MAC_PAGE2 ((uint32_t)0x0800D3FF) /* Base @ of Page 244, 256 bytes */
#define ADDR_FLASH_LORA_REG_PAGE1 ((uint32_t)0x0800D400)
#define ADDR_FLASH_LORA_REG_PAGE2 ((uint32_t)0x0800D7FF)
#define ADDR_FLASH_LORA_REG1_PAGE1 ((uint32_t)0x0800D800)
#define ADDR_FLASH_LORA_REG1_PAGE2 ((uint32_t)0x0800DBFF)
#define ADDR_FLASH_SetMAC_PAGE1 ((uint32_t)0x0800DC00) /* Base @ of Page 240, 256 bytes */
#define ADDR_FLASH_SetMAC_PAGE2 ((uint32_t)0x0800DFFF) /* Base @ of Page 244, 256 bytes */
#define ADDR_FLASH_GetVtg_PAGE1 ((uint32_t)0x0800E000) /* 用于存放电量信息，调试时使用 */
#define ADDR_FLASH_GetVtg_PAGE2 ((uint32_t)0x0800E3FF) /* 用于存放电量信息，调试时使用 */

#define FLASH_SetMAC_START_ADDR       	ADDR_FLASH_SetMAC_PAGE1    	/* Start @ of user Flash area */
#define FLASH_SetMAC_END_ADDR         	ADDR_FLASH_SetMAC_PAGE2    	/* End @ of user Flash area */
#define FLASH_MAC_START_ADDR       		ADDR_FLASH_MAC_PAGE1    	/* Start @ of user Flash area */
#define FLASH_MAC_END_ADDR        	 	ADDR_FLASH_MAC_PAGE2    	/* End @ of user Flash area */
#define FLASH_LORA_REG_START_ADDR	 	ADDR_FLASH_LORA_REG_PAGE1
#define FLASH_LORA_REG_END_ADDR		 	ADDR_FLASH_LORA_REG_PAGE2 
#define FLASH_LORA_REG1_START_ADDR	 	ADDR_FLASH_LORA_REG1_PAGE1
#define FLASH_LORA_REG1_END_ADDR		ADDR_FLASH_LORA_REG1_PAGE2 
#define FLASH_Get_Voltage_START_ADDR	ADDR_FLASH_GetVtg_PAGE1		/* 用于存放电量信息，调试时使用 */
#define FLASH_Get_Voltage_END_ADDR		ADDR_FLASH_GetVtg_PAGE2		/* 用于存放电量信息，调试时使用 */


#define ADDR_FLASH_QUEUE_PAGE1              ((uint32_t)0x0800E400)
#define ADDR_FLASH_QUEUE_PAGE2              ((uint32_t)0x0800E7FF)
#define ADDR_FLASH_PASSWORD_PAGE1           ((uint32_t)0x0800E800)
#define ADDR_FLASH_PASSWORD_PAGE2           ((uint32_t)0x0800EBFF)
#define ADDR_FLASH_LOG_PAGE1                ((uint32_t)0x0800EC00)
#define ADDR_FLASH_LOG_PAGE2                ((uint32_t)0x0800EFFF)

#define FLASH_LOG_START_ADDR	 	             ADDR_FLASH_LOG_PAGE1
#define FLASH_LOG_END_ADDR		               ADDR_FLASH_LOG_PAGE2
#define FLASH_QUEUE_START_ADDR	 	           ADDR_FLASH_QUEUE_PAGE1
#define FLASH_QUEUE_END_ADDR		 	           ADDR_FLASH_QUEUE_PAGE2
#define FLASH_PASSWORD_START_ADDR	 	         ADDR_FLASH_PASSWORD_PAGE1
#define FLASH_PASSWORD_END_ADDR		 	         ADDR_FLASH_PASSWORD_PAGE2

/**
	* @func 电池信息结构体
	*/
typedef struct 
{
	uint32_t  VoltageValue[7];
}BatteryData;

/**
	* @func 设备信息结构体
	*/
typedef struct _st_endevice_info
{
	uint8_t local_addr[MAC_LEN];
	uint8_t server_addr[MAC_GW_LEN];
	uint8_t node_type;
	uint8_t status;
	uint8_t regStatus;
	uint8_t ConfigFlag[2]; //0xaa,0x55
}__attribute__ ((packed))ENDEVICE_INFO_t;

/**
	* @func LoRa寄存器结构体，用于USB配置相关参数
	*/
typedef struct _st_LoRaReg_info
{
	uint8_t VALUE_LR_RegFrMsb;
	uint8_t VALUE_LR_RegFrMid;
	uint8_t VALUE_LR_RegFrLsb;
	uint8_t VALUE_LR_RegModemConfig1;
	uint8_t VALUE_LR_RegModemConfig2;
	uint8_t VALUE_LR_RegModemConfig3;
	uint8_t VALUE_LR_RegPaConfig; 
	uint8_t VALUE_LR_RegPreambleMsb; 
}__attribute__ ((packed))LORAREG_INFO_t;

/**
	* @func lora 结构体
	*/
typedef struct _st_LoRaReg1_info
{
	uint8_t VALUE_LR_RegPreambleLsb; 

}__attribute__ ((packed))LORAREG1_INFO_t;


extern ENDEVICE_INFO_t           g_endevice_info;
extern LORAREG_INFO_t            g_LoRaReg_info;
extern LORAREG1_INFO_t           g_LoRaReg1_info;
extern __align(4) uint8_t      NodeSixByteMac[8];
extern __align(4) BatteryData  g_voltage_value;

uint32_t ekey_flash_read(uint32_t Address, uint32_t Read_len, uint32_t *pData, int FlashFlag);
HAL_StatusTypeDef FLASH_Write(uint32_t Address, uint32_t *pData, uint32_t Data_len);
uint32_t FLASH_Read(uint32_t Address, uint32_t Read_len,uint32_t *pData, int FlashFlag);




#endif
