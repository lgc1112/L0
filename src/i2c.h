#ifndef __BH1750_H_
#define __BH1750_H_

#include "stm32l0xx_hal.h"
#include "stm32l0xx_nucleo.h"

#ifndef BV
#define BV(n)      (1 << (n))
#endif

#if 0
#define st(x)      do { x } while (__LINE__ == -1)
#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin## = val; )
#define HAL_IO_GET(port, pin)   HAL_IO_GET_PREP( port,pin)
#define HAL_IO_GET_PREP(port, pin)   ( P##port##_##pin)



#define LIGHT_SCK_0()        HAL_IO_SET(1,3,0)
#define LIGHT_SCK_1()        HAL_IO_SET(1,3,1)
#define LIGHT_DTA_0()        HAL_IO_SET(1,2,0)
#define LIGHT_DTA_1()        HAL_IO_SET(1,2,1)

#define LIGHT_DTA()          HAL_IO_GET(1,2)
#define LIGHT_SCK()          HAL_IO_GET(1,3)

#define SDA_W() (P1DIR |=BV(2)  )
#define SDA_R() (P1DIR &=~BV(2) )
#endif

#if 1
typedef unsigned char uint8;
#define GPIO_SCL	  GPIO_PIN_8
#define GPIO_SDA  	GPIO_PIN_9
#define IRQ					GPIO_PIN_13

#define NTM_SCL_LOW()        HAL_GPIO_WritePin(GPIOB, GPIO_SCL, GPIO_PIN_RESET)
#define NTM_SCL_HIGH()        HAL_GPIO_WritePin(GPIOB, GPIO_SCL, GPIO_PIN_SET)
#define NTM_SDA_LOW()        HAL_GPIO_WritePin(GPIOB, GPIO_SDA, GPIO_PIN_RESET)
#define NTM_SDA_HIGH()        HAL_GPIO_WritePin(GPIOB, GPIO_SDA, GPIO_PIN_SET)

#define NTM_SDA_READ()          HAL_GPIO_ReadPin(GPIOB, GPIO_SDA)
//#define LIGHT_SCK()          HAL_GPIO_ReadPin(GPIOB, GPIO_SCL)

#define NTM_SDA_OUT 							i2c_outputpin_init(GPIO_SDA) 
#define NTM_SDA_IN 							i2c_inputpin_init(GPIO_SDA)

#define NTM_SCL_OUT								i2c_outputpin_init(GPIO_SCL)

#define NTM_IRQ_INPUT_PU          debugger_init()
#define NTM_IRQ_READ()        HAL_GPIO_ReadPin(GPIOC,IRQ)
#define NOP() delay_us(3)

#define NTM_I2C_ADDR   (0x08<<1) 

#endif
//#define delay() {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}



/*
**  WHO_AM_I Device ID Register
*/
#define WHO_AM_I_REG          0x07
#define MAG3110Q_ID           0xC4    

//MAG3110 ADD
#define IIC_WRITE_DEVICE_ADDRESS       0x1C
#define IIC_READ_DEVICE_ADDRESS        0x1D

#define CTRL_REG1        0x10
#define CTRL_REG2        0x11


#define ACTIVE_MASK           0x01
#define STANDBY_MASK          0x00
//#define DR1_MASK              0x40
//#define DR1_MASK              0x18
//#define DATA_RATE_5MS         DR1_MASK

#define STATUS_00_REG         0x00

#define ZYXOW_MASK            0x80
#define ZOW_MASK              0x40
#define YOR_MASK              0x20
#define XOR_MASK              0x10
#define ZYXDR_MASK            0x08
#define ZDR_MASK              0x04
#define YDR_MASK              0x02
#define XDR_MASK              0x01

/*
**  XYZ Data Registers
*/
#define OUT_X_MSB_REG         0x01
#define OUT_X_LSB_REG         0x02
#define OUT_Y_MSB_REG         0x03
#define OUT_Y_LSB_REG         0x04
#define OUT_Z_MSB_REG         0x05
#define OUT_Z_LSB_REG         0x06   

#define OUT_X_Offset_MSB_REG         0x09
#define OUT_X_Offset_LSB_REG         0x0A
#define OUT_Y_Offset_MSB_REG         0x0B
#define OUT_Y_Offset_LSB_REG         0x0C
#define OUT_Z_Offset_MSB_REG         0x0D
#define OUT_Z_Offset_LSB_REG         0x0E  


extern void i2c_pin_init(uint32_t pin,uint32_t mode);
extern void i2c_Initialize(void);
//extern unsigned short get_light(void);
extern void MAG3110_Init (void);
extern void MAG3110_Standby (void);
extern void MAG3110_Config(void);
extern uint8 MAG3110_ByteWrite(unsigned char RomAddress,unsigned char Data);
extern unsigned char MAG3110_RandomRead(unsigned char RomAddress);



extern char i2c_read(char ack);
extern void 	start_i2c(void);
extern void  stop_i2c(void);
extern void mydelay(void);
extern void mydelaya(void);


#define NTM_I2C_ADDR   (0x08<<1) 

extern int NTM_Read(uint8_t* rdBuf, uint8_t rdLen);
extern int NTM_Write(uint8_t* wrBuf, uint8_t wrLen);
extern uint8_t _i2c_Read_Byte(void);
extern char _i2c_Send_Byte(uint8_t sendbyte);

extern void NTM_Init(void);
extern void debugger_init(void);
extern void DelayMS(uint16_t iMs);
extern uint8_t NTM_ReadRandom(uint8_t* rdBuf);//lgc0724
#endif

