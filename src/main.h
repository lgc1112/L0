 /**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComPolling/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    18-June-2014
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_nucleo.h"
#include "stm32l0xx_hal.h"
#include "lora.h"
#include "bochiot_protocol.h"
#include "uart_debug.h"
#include "i2c.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_customhid.h" 
#include "ekey.h"
#include "timer.h"


#define ON 							0x01
#define OFF 						0x00
#define UartConfigForLora   		OFF		//是否启用串口配置lora寄存器开关
#define	voltagebutton				OFF		//记录电量开关
#define IWDG_SWITCH         	ON//	OFF  ON		//看门狗开关
#define UART2								//串口2的选择开关，禁用则选择串口1

//usb配置LORA寄存器的控制引脚选择   串口版4  原版10
#define GPIO_PIN_usb				GPIO_PIN_8



//节点回包时的前道码长度:500ms
#define preambleLsb_packetback		0x7B
#define preambleMsb_packetback		0x00

#define Electric_Contorl_New        0x01     	//取电开关的控制版本
#define NODE_TYPE                   0x01        //设备类型          锁：1  电控制：0
#define Flash_ctrl                  0x01        //flash开关
#define SelectSystemClock           0x00        //系统时钟选择  0x00：12mhz   0x01：16mhz
#define GwMac                       0x01        //测试flash使用




#define DeviceType                	0x02        //设备类型    0x01：网关  0x02：门锁  0x03：取电开关  现在不需要了
//#define RoomNumber                	0x30				//配置MAC信息

//lgc 0726
#define DevcMAC4                	0x00				//设备的第四位MAC信息
#define DevcMAC5                	0x11				//设备的第五位MAC信息
#define DevcMAC6                	0x11 	    //设备的第六位MAC信息

//节点mac
#define MeasureVoltage              0x00        // 0x01开启电量检测；0x00:关闭检测  
//#define DetectPowerCycle          7000        //电量检测时间控制  现在不需要了

//stop模式时间
//扩频因子  9  12mhz  CPU 时钟 下参数设置
#define StopTime  						0xb80						//stop 时间是 1.5s 0xb80
//#define 	StopTime  				0x750          	//stop 时间是 1s

//扩频因子  7  12mhz  CPU 时钟 下参数设置
//#define StopTime  					0xb80						//stop 时间是 1.5s
//#define StopTime  					0x770          	//stop 时间是 1s









/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor SPIx instance used and associated 
   resources */
/* Definition for SPIx clock resources */
#define SPIx                             SPI2
#define SPIx_CLK_ENABLE()                __SPI2_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE() 
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE() 
#define SPIx_NSS_GPIO_CLK_ENABLE()       __GPIOB_CLK_ENABLE() 
#define PowerSwitch_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE() 



#define SPIx_FORCE_RESET()               __SPI2_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __SPI2_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_13
#define SPIx_SCK_GPIO_PORT               GPIOB
#define SPIx_SCK_AF                      GPIO_AF0_SPI2
#define SPIx_MISO_PIN                    GPIO_PIN_14
#define SPIx_MISO_GPIO_PORT              GPIOB
#define SPIx_MISO_AF                     GPIO_AF0_SPI2
#define SPIx_MOSI_PIN                    GPIO_PIN_15
#define SPIx_MOSI_GPIO_PORT              GPIOB
#define SPIx_MOSI_AF                     GPIO_AF0_SPI2

//NSS configuration
#define SPIx_NSS_PIN                     GPIO_PIN_12
#define SPIx_NSS_GPIO_PORT               GPIOB
#define SPIx_NSS_AF                      GPIO_AF0_SPI2


//LORA RX LED configuration
#define LORA_RXLED_PIN                   GPIO_PIN_9
#define LORA_RXLED_GPIO_PORT             GPIOB


//?a1?μ??ìμ??÷????GPIO
#define LORA_PowerSwitch_PIN_7           	GPIO_PIN_7
#define LORA_PowerSwitch_PIN_6           	GPIO_PIN_6
#define LORA_PowerSwitch_GPIO_PORT      	GPIOA
//Clock   configuration PIN
#define CLOCK_PIN_8                  		GPIO_PIN_9      //lgc 0723
#define CLOCK_GPIO_PORT_8           		GPIOA
#define CLOCK_PIN_9                 		GPIO_PIN_10
#define CLOCK_GPIO_PORT_9             		GPIOA 
#define CLOCK_PIN_BEEP                 	GPIO_PIN_0
#define CLOCK_GPIO_PORT_BEEP            GPIOA           //lgc 0723//lgc 0723

//°′?￥??ó|μ?GPIOòy??
#define BUTTON_PIN_0                  	GPIO_PIN_10
#define BUTTON_GPIO_PORT_0           		GPIOB
//Lora reset configuration
#define LORA_RESET_PIN                   GPIO_PIN_15
#define LORA_RESET_GPIO_PORT             GPIOA

//Clock   configuration PIN
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the corresponding line to select the RTC Clock source */
#define RTC_CLOCK_SOURCE_LSE 
/*#define RTC_CLOCK_SOURCE_LSI */ /* LSI used as RTC source clock. The RTC Clock
                                      may varies due to LSI frequency dispersion. */

#ifdef RTC_CLOCK_SOURCE_LSI
  #define RTC_ASYNCH_PREDIV    0x7F
  #define RTC_SYNCH_PREDIV     0x0130
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
  #define RTC_ASYNCH_PREDIV  0x7F
  #define RTC_SYNCH_PREDIV   0x00FF
#endif

/* Uncomment the line below to select your USB clock source */
#define USE_USB_CLKSOURCE_CRSHSI48   1
//#define USE_USB_CLKSOURCE_PLL        1

#if !defined (USE_USB_CLKSOURCE_PLL) && !defined (USE_USB_CLKSOURCE_CRSHSI48)
 #error "Missing USB clock definition"
#endif




/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated 
   resources */
#ifdef UART2
/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __USART2_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA  
#define USARTx_TX_AF                     GPIO_AF4_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA 
#define USARTx_RX_AF                     GPIO_AF4_USART2
#else
/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __USART1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA  
#define USARTx_TX_AF                     GPIO_AF4_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA 
#define USARTx_RX_AF                     GPIO_AF4_USART1
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Base address of the Flash pages */
#define ADDR_FLASH_PAGE_0   ((uint32_t)0x08000000) /* Base @ of Page 0, 256 bytes */
#define ADDR_FLASH_PAGE_1   ((uint32_t)0x08000100) /* Base @ of Page 1, 256 bytes */
#define ADDR_FLASH_PAGE_2   ((uint32_t)0x08000200) /* Base @ of Page 2, 256 bytes */
#define ADDR_FLASH_PAGE_3   ((uint32_t)0x08000300) /* Base @ of Page 3, 256 bytes */
#define ADDR_FLASH_PAGE_4   ((uint32_t)0x08000400) /* Base @ of Page 4, 256 bytes */
#define ADDR_FLASH_PAGE_5   ((uint32_t)0x08000500) /* Base @ of Page 5, 256 bytes */
#define ADDR_FLASH_PAGE_6   ((uint32_t)0x08000600) /* Base @ of Page 6, 256 bytes */
#define ADDR_FLASH_PAGE_7   ((uint32_t)0x08000700) /* Base @ of Page 7, 256 bytes */
#define ADDR_FLASH_PAGE_8   ((uint32_t)0x08000800) /* Base @ of Page 8, 256 bytes */
#define ADDR_FLASH_PAGE_9   ((uint32_t)0x08000900) /* Base @ of Page 9, 256 bytes */
#define ADDR_FLASH_PAGE_10  ((uint32_t)0x08000A00) /* Base @ of Page 10, 256 bytes */
#define ADDR_FLASH_PAGE_11  ((uint32_t)0x08000B00) /* Base @ of Page 11, 256 bytes */
#define ADDR_FLASH_PAGE_12  ((uint32_t)0x08000C00) /* Base @ of Page 12, 256 bytes */
#define ADDR_FLASH_PAGE_13  ((uint32_t)0x08000D00) /* Base @ of Page 13, 256 bytes */
#define ADDR_FLASH_PAGE_14  ((uint32_t)0x08000E00) /* Base @ of Page 14, 256 bytes */
#define ADDR_FLASH_PAGE_15  ((uint32_t)0x08000F00) /* Base @ of Page 15, 256 bytes */
#define ADDR_FLASH_PAGE_16  ((uint32_t)0x08001000) /* Base @ of Page 16, 256 bytes */
#define ADDR_FLASH_PAGE_17  ((uint32_t)0x08001100) /* Base @ of Page 17, 256 bytes */
#define ADDR_FLASH_PAGE_18  ((uint32_t)0x08001200) /* Base @ of Page 18, 256 bytes */
#define ADDR_FLASH_PAGE_19  ((uint32_t)0x08001300) /* Base @ of Page 19, 256 bytes */
#define ADDR_FLASH_PAGE_20  ((uint32_t)0x08001400) /* Base @ of Page 20, 256 bytes */
#define ADDR_FLASH_PAGE_21  ((uint32_t)0x08001500) /* Base @ of Page 21, 256 bytes */
#define ADDR_FLASH_PAGE_22  ((uint32_t)0x08001600) /* Base @ of Page 22, 256 bytes */
#define ADDR_FLASH_PAGE_23  ((uint32_t)0x08001700) /* Base @ of Page 23, 256 bytes */
#define ADDR_FLASH_PAGE_24  ((uint32_t)0x08001800) /* Base @ of Page 24, 256 bytes */
#define ADDR_FLASH_PAGE_25  ((uint32_t)0x08001900) /* Base @ of Page 25, 256 bytes */
#define ADDR_FLASH_PAGE_26  ((uint32_t)0x08001A00) /* Base @ of Page 26, 256 bytes */
#define ADDR_FLASH_PAGE_27  ((uint32_t)0x08001B00) /* Base @ of Page 27, 256 bytes */
#define ADDR_FLASH_PAGE_28  ((uint32_t)0x08001C00) /* Base @ of Page 28, 256 bytes */
#define ADDR_FLASH_PAGE_29  ((uint32_t)0x08001D00) /* Base @ of Page 29, 256 bytes */
#define ADDR_FLASH_PAGE_30  ((uint32_t)0x08001E00) /* Base @ of Page 30, 256 bytes */
#define ADDR_FLASH_PAGE_31  ((uint32_t)0x08001F00) /* Base @ of Page 31, 256 bytes */
#define ADDR_FLASH_PAGE_32  ((uint32_t)0x08002000) /* Base @ of Page 32, 256 bytes */
#define ADDR_FLASH_PAGE_33  ((uint32_t)0x08002100) /* Base @ of Page 33, 256 bytes */
#define ADDR_FLASH_PAGE_34  ((uint32_t)0x08002200) /* Base @ of Page 34, 256 bytes */
#define ADDR_FLASH_PAGE_35  ((uint32_t)0x08002300) /* Base @ of Page 35, 256 bytes */
#define ADDR_FLASH_PAGE_36  ((uint32_t)0x08002400) /* Base @ of Page 36, 256 bytes */
#define ADDR_FLASH_PAGE_37  ((uint32_t)0x08002500) /* Base @ of Page 37, 256 bytes */
#define ADDR_FLASH_PAGE_38  ((uint32_t)0x08002600) /* Base @ of Page 38, 256 bytes */
#define ADDR_FLASH_PAGE_39  ((uint32_t)0x08002700) /* Base @ of Page 39, 256 bytes */
#define ADDR_FLASH_PAGE_40  ((uint32_t)0x08002800) /* Base @ of Page 40, 256 bytes */
#define ADDR_FLASH_PAGE_41  ((uint32_t)0x08002900) /* Base @ of Page 41, 256 bytes */
#define ADDR_FLASH_PAGE_42  ((uint32_t)0x08002A00) /* Base @ of Page 42, 256 bytes */
#define ADDR_FLASH_PAGE_43  ((uint32_t)0x08002B00) /* Base @ of Page 43, 256 bytes */
#define ADDR_FLASH_PAGE_44  ((uint32_t)0x08002C00) /* Base @ of Page 44, 256 bytes */
#define ADDR_FLASH_PAGE_45  ((uint32_t)0x08002D00) /* Base @ of Page 45, 256 bytes */
#define ADDR_FLASH_PAGE_46  ((uint32_t)0x08002E00) /* Base @ of Page 46, 256 bytes */
#define ADDR_FLASH_PAGE_47  ((uint32_t)0x08002F00) /* Base @ of Page 47, 256 bytes */
#define ADDR_FLASH_PAGE_48  ((uint32_t)0x08003000) /* Base @ of Page 48, 256 bytes */
#define ADDR_FLASH_PAGE_49  ((uint32_t)0x08003100) /* Base @ of Page 49, 256 bytes */
#define ADDR_FLASH_PAGE_50  ((uint32_t)0x08003200) /* Base @ of Page 50, 256 bytes */
#define ADDR_FLASH_PAGE_51  ((uint32_t)0x08003300) /* Base @ of Page 51, 256 bytes */
#define ADDR_FLASH_PAGE_52  ((uint32_t)0x08003400) /* Base @ of Page 52, 256 bytes */
#define ADDR_FLASH_PAGE_53  ((uint32_t)0x08003500) /* Base @ of Page 53, 256 bytes */
#define ADDR_FLASH_PAGE_54  ((uint32_t)0x08003600) /* Base @ of Page 54, 256 bytes */
#define ADDR_FLASH_PAGE_55  ((uint32_t)0x08003700) /* Base @ of Page 55, 256 bytes */
#define ADDR_FLASH_PAGE_56  ((uint32_t)0x08003800) /* Base @ of Page 56, 256 bytes */
#define ADDR_FLASH_PAGE_57  ((uint32_t)0x08003900) /* Base @ of Page 57, 256 bytes */
#define ADDR_FLASH_PAGE_58  ((uint32_t)0x08003A00) /* Base @ of Page 58, 256 bytes */
#define ADDR_FLASH_PAGE_59  ((uint32_t)0x08003B00) /* Base @ of Page 59, 256 bytes */
#define ADDR_FLASH_PAGE_60  ((uint32_t)0x08003C00) /* Base @ of Page 60, 256 bytes */
#define ADDR_FLASH_PAGE_61  ((uint32_t)0x08003D00) /* Base @ of Page 61, 256 bytes */
#define ADDR_FLASH_PAGE_62  ((uint32_t)0x08003E00) /* Base @ of Page 62, 256 bytes */
#define ADDR_FLASH_PAGE_63  ((uint32_t)0x08003F00) /* Base @ of Page 63, 256 bytes */
#define ADDR_FLASH_PAGE_64  ((uint32_t)0x08004000) /* Base @ of Page 64, 256 bytes */

#define ADDR_FLASH_MY_PAGE3 ((uint32_t)0x0800D400) /* Base @ of Page 240, 256 bytes */
#define ADDR_FLASH_MY_PAGE4 ((uint32_t)0x0800D800) /* Base @ of Page 244, 256 bytes */

#define FLASH_USER_START_ADDR1       ADDR_FLASH_MY_PAGE3   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR1         ADDR_FLASH_MY_PAGE4   /* End @ of user Flash area */




void bochiot_send_msg(uint8 *msg, uint8 msg_len);
int LORA_RX(void);
void open_lock(void);
#endif /* __MAIN_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
