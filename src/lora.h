

/********************************************************************/
#include  <stdbool.h>
//#include "stm32l1xx_nucleo.h"

#define	UNLIMIT_TEST			//作为无限测试模式

typedef unsigned char uint8;
typedef unsigned char byte;
typedef unsigned int  word;
typedef unsigned long lword;

extern volatile uint8 Tx_Count; //发送失败的数据包个数

/**********************************************************
**变量声明
**********************************************************/


//常量定义
#define	Coding_Rate_Value					0x01			//纠错编码率
#define	CRC_ENBLE							0x04			//CRC 开启
#define	CRC_NO								0x00			//关闭CRC

//BW 设置
#define		Value_BWSel_7800Hz				0		//7.8KHz
#define		Value_BWSel_10400Hz				1		//10.4KHz
#define		Value_BWSel_15600Hz				2		//15.6KHz
#define		Value_BWSel_20800Hz				3		//20.8KHz
#define		Value_BWSel_31250Hz				4		//31.25KHz
#define		Value_BWSel_41700Hz				5		//41.7KHz
#define		Value_BWSel_62500Hz				6		//62.5KHz
#define		Value_BWSel_125KHz				7		//125KHz    ( 默认值)
#define		Value_BWSel_200KHz				8		//200KHz
#define		Value_BWSel_500KHz				9		//500KHz

//扩频因子Spreading Factor (SF)设置
#define		Value_SFactor_7				7		//扩频因子是7
#define		Value_SFactor_8				8		//扩频因子是8
#define		Value_SFactor_9				9		//扩频因子是9  ( 默认值)
#define		Value_SFactor_10			10		//扩频因子是10
#define		Value_SFactor_11			11		//扩频因子是11
#define		Value_SFactor_12			12		//扩频因子是12

// SX1276/77/78 Internal registers Address
//通用寄存器
#define LR_RegFifo                                  0x00	// FIFO数据输入输出地址寄存器
#define LR_RegOpMode                                0x01	//模式寄存器
#define LR_RegFrMsb                                 0x06	//RF载波频率最高有效位寄存器
#define LR_RegFrMid                                 0x07	//RF载波频率中间有效位寄存器
#define LR_RegFrLsb                                 0x08	//RF载波频率最低有效位寄存器

// RF模块寄存器(TX)
#define LR_RegPaConfig                              0x09	//PA配置寄存器
#define LR_RegPaRamp                                0x0A	//LORA模式下斜升/斜降时间配置寄存器
#define LR_RegOcp                                   0x0B	//OCP配置寄存器

//(TX)
#define LR_RegLna                                   0x0C	//LNA配置寄存器

//定序寄存器
#define LR_ReqSeqConfig1			0x36    //顶级定序器设置

// LoRa 页面寄存器
#define LR_RegFifoAddrPtr                           0x0D	//FIFO数据缓冲区中SPI接口地址指针寄存器
#define LR_RegFifoTxBaseAddr                        0x0E	//FIFO数据缓冲区中发送调试器的写入基地址寄存器
#define LR_RegFifoRxBaseAddr                        0x0F	//FIFO数据缓冲区中接收调试器的读取基地址寄存器
#define LR_RegFifoRxCurrentaddr                     0x10	//接收到最后一个数据包的起始地址（数据缓冲区中）寄存器
#define LR_RegIrqFlagsMask                          0x11	//中断位置掩码配置寄存器
#define	LR_RegIrqFlags								0x12	//中断标记值寄存器
#define	LR_RegRxNbBytes								0x13	//最近一次接收到的数据包的负载字节数指示寄存器
#define	LR_RegRxHeaderCntValueMsb					0x14	//最后一次转换至Rx 模式后接收的有效报头数指示寄存器
#define	LR_RegRxHeaderCntValueLsb					0x15	//最后一次转换至Rx 模式后接收的有效报头数指示寄存器
#define	LR_RegRxPacketCntValueMsb					0x16	//最后一次转换至Rx 模式后接收的有效数据包数指示寄存器
#define	LR_RegRxPacketCntValueLsb					0x17	//最后一次转换至Rx 模式后接收的有效数据包数指示寄存器
#define	LR_RegModemStat								0x18	//调制器状态指示寄存器
#define	LR_RegPktSnrValue							0x19	//最后接收到的数据包的SNR 预估值指示寄存器
#define	LR_RegPktRssiValue							0x1A	//最后接收到的数据包的RSSI（dBm）指示寄存器
#define	LR_RegRssiValue								0x1B	//电流RSSI 值（dBm）指示寄存器
#define	LR_RegHopChannel							0x1C	//hop信道配置寄存器
#define	LR_RegModemConfig1							0x1D	//调制器参数配置寄存器1
#define	LR_RegModemConfig2							0x1E	//调制器参数配置寄存器2
#define	LR_RegSymbTimeoutLsb						0x1F	//RX 超时最低有效位指示寄存器
#define	LR_RegPreambleMsb							0x20	//前导码长度最高有效位指示寄存器
#define	LR_RegPreambleLsb							0x21	//前导码长度最低有效位指示寄存器
#define	LR_RegPayloadLength							0x22	//负载字节长度配置寄存器
#define	LR_RegMaxPayloadLength						0x23	//负载长度最大值配置寄存器
#define	LR_RegHopPeriod								0x24	//频率跳变之间的符号周期配置寄存器
#define	LR_RegFifoRxByteAddr						0x25	//接收数据缓存当前指针配置寄存器
#define	LR_RegModemConfig3							0x26	//调制器参数配置寄存器3


// I/O settings
#define LR_RegDIOMAPPING1                          0x40		//DIO0 到DIO5 引脚的映射配置寄存器1
#define LR_RegDIOMAPPING2                          0x41		//DIO0 到DIO5 引脚的映射配置寄存器2
// Version
#define LR_RegVERSION                              0x42		//芯片版本号指示寄存器

// 附加寄存器
//#define LR_RegPLLHOP                               			0x44		//PLL调频配置寄存器
#define LR_RegTCXO                                 0x4B		//控制晶体振荡器配置寄存器
#define LR_RegPADAC                                0x4D		//在PA_BOOST 引脚上启动+20dBm 选项配置寄存器
#define LR_RegFORMERTEMP                           0x5B		//前导码长度最低有效位最后一次IQ（RSSI 和镜像）校准中保存的温度指示寄存器
#define LR_RegAGCREF                               0x61		//为所有AGC 阈值设置最低参考值配置寄存器
#define LR_RegAGCTHRESH1                           0x62		//定义第一个AGC 阈值配置寄存器
#define LR_RegAGCTHRESH2                           0x63		//定义第二个AGC 阈值配置寄存器
#define LR_RegAGCTHRESH3                           0x64		//定义第三个AGC 阈值配置寄存器
#define LR_RegAGCTHRESH3                           0x64		//定义第三个AGC 阈值配置寄存器
#define LR_RegPll                           	   0x70		//PLL带宽控制寄存器 


//模式的值信息
#define VALUE_RegOpMode_SLEEP                       0x88		//Sleep 模式(都开启低频参数)
#define VALUE_RegOpMode_LORA                        0x88		//Lora 模式(都开启低频参数)
#define VALUE_RegOpMode_STANDBY                     0x89		//Stand by 模式
#define VALUE_RegOpMode_ContinuousRx              	0x05+0x08	//Continuous Rx Mode and Low frequency
#define VALUE_RegOpMode_Tx              			0x8B		//0x80+0x03+0x08	//TX Mode and Low frequency
#define VALUE_RegOpMode_CAD                         0x87     //CAD模式

//寄存器的默认值信息
//#define VALUE_LR_RegFrMsb                           		0x6C	// 0x06:  RF载波频率最高有效位的默认值
//#define VALUE_LR_RegFrMid                           		0X80	//0X80	//0x40	//0x07:  RF载波频率中间有效位的默认值
//#define VALUE_LR_RegFrLsb                           		0x00	//0x08:  RF载波频率最低有效位的默认值
//#define VALUE_LR_RegPaConfig                        		0x8F	//0x8F	//0xFF	//0x09:  PA配置寄存器的默认值

#define VALUE_LR_RegPaRamp                                	0x09	//0x0A:  LORA模式下斜升/斜降时间配置寄存器的默认值
#define VALUE_LR_RegOcp                                   	0x2F	//0x0B:  OCP配置寄存器的默认值

//(TX)
#define VALUE_LR_RegLna                                   	0X20	//0x23	//0x0C:  LNA配置寄存器的默认值

//定序寄存器
#define VALUE_LR_ReqSeqConfig1				0x20      //设置定序器为睡眠模式
 

// LoRa 页面寄存器
#define VALUE_LR_RegFifoAddrPtr                           	0x00	//0x0D:  FIFO数据缓冲区中SPI接口地址指针寄存器的默认值
#define VALUE_LR_RegFifoTxBaseAddr                        	0x80	//0x0E:  FIFO数据缓冲区中发送调试器的写入基地址寄存器的默认值
#define VALUE_LR_RegFifoRxBaseAddr                       	0x00	//0x0F:  FIFO数据缓冲区中接收调试器的读取基地址寄存器的默认值
#define VALUE_LR_RegFifoRxCurrentaddr                     	0x10	// n/a    //0x10:  接收到最后一个数据包的起始地址（数据缓冲区中）寄存器的默认值
#define VALUE_LR_RegIrqFlagsMask							0x00	//启动时候的默认值 0x11:  中断位置掩码配置寄存器的默认值
#define VALUE_LR_RegIrqFlagsMask_RX                         0x3F	//0x11:  中断位置掩码配置寄存器的默认值
#define VALUE_LR_RegIrqFlagsMask_TX                         0xF7	//0x11:  中断位置掩码配置寄存器的默认值
#define VALUE_LR_RegIrqFlagsMask_CAD                        0xFA  //0x11:  中断位置掩码配置寄存器的默认值   
#define VALUE_LR_RegIrqFlagsMask_CRT                        0x1F  //同时开启RxTIMEOUT TxDONE CRC中断  0x11:  中断位置掩码配置寄存器的默认值
#define	VALUE_LR_RegIrqFlags								0xFF	//0xFF	0x00 //0x12:  中断标记值寄存器的默认值
#define	VALUE_LR_RegRxNbBytes								0x13	//n/a     0x13:  最近一次接收到的数据包的负载字节数指示寄存器的默认值
#define	VALUE_LR_RegRxHeaderCntValueMsb						0x14	//n/a     0x14:  最后一次转换至Rx 模式后接收的有效报头数指示寄存器的默认值
#define	VALUE_LR_RegRxHeaderCntValueLsb						0x15	//n/a     0x15:  最后一次转换至Rx 模式后接收的有效报头数指示寄存器的默认值
#define	VALUE_LR_RegRxPacketCntValueMsb						0x16	//n/a     0x16:  最后一次转换至Rx 模式后接收的有效数据包数指示寄存器的默认值
#define	VALUE_LR_RegRxPacketCntValueLsb						0x17	//n/a     0x17:  最后一次转换至Rx 模式后接收的有效数据包数指示寄存器的默认值
#define	VALUE_LR_RegModemStat								0x18	//n/a     0x18:  调制器状态指示寄存器的默认值
#define	VALUE_LR_RegPktSnrValue								0x19	//n/a     0x19:  最后接收到的数据包的SNR 预估值指示寄存器的默认值
#define	VALUE_LR_RegPktRssiValue							0x1A	//n/a     0x1A:  最后接收到的数据包的RSSI（dBm）指示寄存器的默认值
#define	VALUE_LR_RegRssiValue								0x1B	//n/a     0x1B:  电流RSSI 值（dBm）指示寄存器的默认值
#define	VALUE_LR_RegHopChannel								0x1C	//n/a     0x1C:  hop信道配置寄存频哪现调
//#define	VALUE_LR_RegModemConfig1							0x72	//0x72	//(Value_BWSel_125KHz<<4)+(Coding_Rate_Value<<1)	//(Value_BWSel_125KHz<<4)+(Coding_Rate_Value<<1)		//0x1D:  调制器参数配置寄存器1的默认值
//#define	VALUE_LR_RegModemConfig2							0x94	//0x94	(Value_SFactor_9<<4)+CRC_ENBLE+0x03	//(Value_SFactor_9<<4)+CRC_ENBLE+0x03	//0x1E:  调制器参数配置寄存器2的默认值
#define	VALUE_LR_RegSymbTimeoutLsb							0xFF	//0x64	//0xFF	//0x1F:  RX 超时最低有效位指示寄存器的默认值
//#define	VALUE_LR_RegPreambleMsb								0		//0x20:   前导码长度最高有效位指示寄存器的默认值
//#define	VALUE_LR_RegPreambleLsb								8		//0x21:   前导码长度最低有效位指示寄存器
#define	VALUE_LR_RegPayloadLength							40		//10		//40		//0x22:   负载字节长度配置寄存器
#define	VALUE_LR_RegMaxPayloadLength						0xFF	//0x23:   负载长度最大值配置寄存器
#define	VALUE_LR_RegHopPeriod								0		//0x24:   频率跳变之间的符号周期配置寄存器
#define	VALUE_LR_RegFifoRxByteAddr							0x25	//n/a     0x25:   接收数据缓存当前指针配置寄存器
//#define	VALUE_LR_RegModemConfig3							0x00	//0x26:   调制器参数配置寄存器3
//#define	VALUE_LR_RegModemConfig3_RX							0x08	//0x26:   调制器参数配置寄存器3


// I/O settings
#define VALUE_LR_RegDIOMAPPING1                         	0x00	//启动时候默认配置 0x40:   DIO0 到DIO5 引脚的映射配置寄存器1
#define VALUE_LR_RegDIOMAPPING1_RX                          0x02	//0x40:   DIO0 到DIO5 引脚的映射配置寄存器1
#define VALUE_LR_RegDIOMAPPING1_TX                         	0x41	//0x40:   DIO0 到DIO5 引脚的映射配置寄存器1
#define VALUE_LR_RegDIOMAPPING1_CAD													0xAF  //0x40:   DIO0 到DIO5 引脚的映射配置寄存器1

#define VALUE_LR_RegDIOMAPPING2                         	0x00	//0x41:   DIO0 到DIO5 引脚的映射配置寄存器2
// Version
#define VALUE_LR_RegVERSION                              	0x12		//芯片版本号指示寄存器

// 附加寄存器
#define VALUE_LR_RegPLLHOP                               	0x44		//0x44:   PLL调频配置寄存器
#define VALUE_LR_RegTCXO                                 	0x09		//0x4B:   控制晶体振荡器配置寄存器
#define VALUE_LR_RegPADAC                             		0x87		//0x4D:   默认配置 在PA_BOOST 引脚上启动+20dBm 选项配置寄存器
#define VALUE_LR_RegPADAC_RX                             	0x84		//0x4D:   在PA_BOOST 引脚上启动+20dBm 选项配置寄存器
#define VALUE_LR_RegPADAC_TX                             	0x87		//0x4D:   在PA_BOOST 引脚上启动+20dBm 选项配置寄存器
#define VALUE_LR_RegFORMERTEMP                           	0x5B		///n/a      0x5B:   前导码长度最低有效位最后一次IQ（RSSI 和镜像）校准中保存的温度指示寄存器
#define VALUE_LR_RegAGCREF                               	0x13		//0x61:   为所有AGC 阈值设置最低参考值配置寄存器
#define VALUE_LR_RegAGCTHRESH1                           	0x0E		//0x62:   定义第一个AGC 阈值配置寄存器
#define VALUE_LR_RegAGCTHRESH2                           	0x5B		//0x63:   定义第二个AGC 阈值配置寄存器
#define VALUE_LR_RegAGCTHRESH3                           	0xDB		//0x64:   定义第三个AGC 阈值配置寄存器
#define VALUE_LR_RegPll										0xD0		//0x70:   PLL带宽控制寄存器 


//需要修改的几个特殊的寄存器     工作频率      前导码     扩频因子      
//默认频率信息寄存器值   /*******0x6C 0X40:  433   0x6C0 X80:  434
#define Default_VALUE_LR_RegFrMsb                           				0x6C	// 0x06:  RF载波频率最高有效位的默认值
#define Default_VALUE_LR_RegFrMid                           				0X80	//0X80	//0x40	//0x07:  RF载波频率中间有效位的默认值  
#define Default_VALUE_LR_RegFrLsb                           				0x00	//0x08:  RF载波频率最低有效位的默认值

//默认前导码长度信息   根据待机时间和扩频因子的值来进行修改   因子：9  待机时间 1S		
//#define	Default_VALUE_LR_RegPreambleMsb															0					//0x20:   前导码长度最高有效位指示寄存器的默认值
//#define	Default_VALUE_LR_RegPreambleLsb															0xF4			//0x21:   前导码长度最低有效位指示寄存器	 				/*******配置前导码长度

//默认前导码长度信息   根据待机时间和扩频因子的值来进行修改   因子：9  待机时间 1.5 S
#define	Default_VALUE_LR_RegPreambleMsb															0x01			//0x20:   前导码长度最高有效位指示寄存器的默认值
#define	Default_VALUE_LR_RegPreambleLsb															0x6E			//0x21:   前导码长度最低有效位指示寄存器	 				/*******配置前导码长度


//默认的扩频因子的值  						//扩频因子9： 0x72 0x94   扩频因子7： 0x72 0x74
#define	Default_VALUE_LR_RegModemConfig1														0x72	  	//调制器参数配置寄存器1的默认值
#define	Default_VALUE_LR_RegModemConfig2														0x94		 	//调制器参数配置寄存器2的默认值

//enum 
//{ 
//NODE_Tx,
//GateWay_Tx
//}
//Tx_Type;



