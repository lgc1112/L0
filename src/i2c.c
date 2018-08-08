#include <stdio.h>
#include "i2c.h"

void delay_nus(void);

void start_i2c(void);
void stop_i2c(void);
char i2c_send(unsigned char val);
char i2c_read(char ack);
void NTM_Init (void);

//unsigned short get_light(void);
//extern void delay_us(int us);

void delay_us(int us)
{
	int i=0;
	while(us--)
	{
		i=10;
		while(i--);
	}
}

void delay_nus(void)
{       
#if 0
        int i;
        int n=100;
        for(i=0;i<n;i++)//延迟32NOP为1US
        {
          asm("nop");asm("nop");asm("nop");asm("nop");
          asm("nop");asm("nop");asm("nop");asm("nop");
          asm("nop");asm("nop");asm("nop");asm("nop");
          asm("nop");asm("nop");asm("nop");asm("nop");
          asm("nop");asm("nop");asm("nop");asm("nop");
          asm("nop");asm("nop");asm("nop");asm("nop");
          asm("nop");asm("nop");asm("nop");asm("nop");
          asm("nop");asm("nop");asm("nop");asm("nop");
        }
#endif
	delay_us(100);
	
}

//ESLAB 
void i2c_inputpin_init(uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	__GPIOB_CLK_ENABLE();
	
	GPIO_InitStructure.Speed=GPIO_SPEED_FAST;//GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Mode= GPIO_MODE_INPUT;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin= pin;
	//GPIO_InitStructure.Alternate = GPIO_AF4_I2C1;


	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}


//ESLAB
 void i2c_outputpin_init(uint16_t pin)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	__GPIOB_CLK_ENABLE();
	
	GPIO_InitStructure.Speed=GPIO_SPEED_FAST;//GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Mode= GPIO_MODE_OUTPUT_OD;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin= pin;




	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

}

//ESLAB
void debugger_init(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	__GPIOC_CLK_ENABLE();
	GPIO_InitStructure.Speed=GPIO_SPEED_FAST;
	GPIO_InitStructure.Pin = GPIO_PIN_13;
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	  /* Enable and set EXTI4_15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}



//void i2c_Initialize(void)
//{
//	i2c_pin_init(GPIO_SCL,GPIO_MODE_OUTPUT_PP);
//	i2c_pin_init(GPIO_SDA,GPIO_MODE_OUTPUT_PP);
//}

/****************************
启动I2C
数据在时钟高电平的时候从高往低跃变

*****************************/

//ESLAB
void start_i2c(void)
{
    NTM_SDA_OUT;

	NTM_SCL_HIGH();
    NOP();NOP();
	NTM_SDA_HIGH(); 
	NOP();NOP();
	NTM_SDA_LOW();  
	NOP(); NOP(); 
}


/********************************

结束I2C

数据在时钟高电平的时候从低往高跃变
********************************/

// ESLAB
void stop_i2c(void)
{
 NTM_SDA_OUT;

	NTM_SDA_LOW();
	NOP(); NOP();
	NTM_SCL_HIGH(); 
	NOP();NOP();
	NTM_SDA_HIGH();
	NOP();	NOP();NOP();

}

/******************************
发送字节并且判断是否收到ACK
当收到ACK返回为0，否则返回为1

******************************/

// ESLAB
char i2c_send(unsigned char val)                 
{
        int i;
        char error=0;
        NTM_SDA_OUT;
        for(i=0x80;i>0;i/=2)
        {
         	if(val & i)
            	NTM_SDA_HIGH();
          else
              NTM_SDA_LOW();
            mydelay();
            NTM_SCL_HIGH() ;
            mydelay();
            NTM_SCL_LOW() ;
            mydelay();           
        }
       
        NTM_SDA_HIGH();
        NTM_SDA_READ();
        //mydelay();
        NTM_SCL_HIGH();
        mydelay();
        if(NTM_SDA_READ())
          error=1;
        mydelay();
        NTM_SCL_LOW() ;
        return error;
       
}

/***************************

读取I2C的字节，并且发送ACK
当参数为1的时候发送一个ACK(低电平)


***************************/
// ESLAB
char i2c_read(char ack)
{
        int i;
        char val=0;
        NTM_SDA_HIGH(); 
        for(i=0x80;i>0;i/=2)
        {             
          NTM_SCL_HIGH();
          mydelay();
          NTM_SDA_IN;
          if(NTM_SDA_READ())
            val=(val|i);
          mydelay();
          NTM_SCL_LOW() ;
          mydelay();      
        }
        NTM_SDA_OUT; //
				
        if(ack) 
          NTM_SDA_LOW();//第一次读接收到一个字节数据后，发送ack信号给从机，还要继续读数据
        else
          NTM_SDA_HIGH(); //第二次读接收到一个字节数据后光照数据都读完了，所以不回ack，从机接收不到ack就认为传输结束

		  
        mydelay(); 	//延时100微秒
        NTM_SCL_HIGH() ; //时钟高低的切换
        mydelay();
        NTM_SCL_LOW() ;
        NTM_SDA_HIGH();
        return val;
}

// ESLAB
void mydelay(void)
{
  uint8_t i;
  for(i=8;i>0;i--){__NOP();}
}

// ESLAB
void mydelaya(void)
{
  uint8_t i;
  for(i=4;i>0;i--){__NOP();}
}


//ESLAB
void _i2c_SendAck(){

  NTM_SDA_OUT;

	NTM_SCL_LOW();
	NOP();
	NTM_SDA_LOW();
	NOP();
	NTM_SCL_HIGH();
  NOP();
  NTM_SCL_LOW();


}

//ESLAB
void _i2c_SendNoack(){
   NTM_SDA_OUT;

	NTM_SCL_LOW();
	NOP();
	NTM_SDA_HIGH();
	NOP();
	NTM_SCL_HIGH();
    NOP();
    NTM_SCL_LOW();

}



//ESLAB
  char _i2c_Send_Byte(uint8_t sendbyte){
	uint8_t ack;
	uint8_t  count;
     
	NTM_SCL_LOW();
	for(count=0; count<8; count++)
	{	
	    NOP(); 
		if ( sendbyte &0x80 ) 
		{ 
			NTM_SDA_HIGH();
		}
		else 
		{ 
			NTM_SDA_LOW();
		}
        sendbyte = sendbyte <<1;
		NOP();
		//NTM_SCL = 1;  // 高脉冲后从机读数据，而从机吐数据是在下降沿
		NTM_SCL_HIGH();
		NOP();	
		//NTM_SCL = 0;   //低电平变换数据
		NTM_SCL_LOW();
	}
    NTM_SDA_IN; // must need 
	
	NOP();
	NTM_SCL_HIGH();
	NOP();
	if(NTM_SDA_READ()) //NTM_SDA)
	{
		ack = 1;
	}
	else
	{	
        ack = 0;
	}
	NTM_SCL_LOW();
	NOP();
	NTM_SDA_OUT; // must need 
	NOP();
	return ack;

}



// ESLAB
uint8_t _i2c_Read_Byte( void){
	
uint8_t  count;
    uint8_t  data_buffer=0;
	NTM_SDA_IN; // must need 
	for(count=0; count<8; count++)
	{  
		NTM_SDA_HIGH();
		NOP();
		//NTM_SCL =1;
		NTM_SCL_HIGH();
		NOP();
		data_buffer <<= 1;
		if (NTM_SDA_READ()) // NTM_SDA ) 
		data_buffer++; 
        //NTM_SCL =0;
		NTM_SCL_LOW();
	}
	return (data_buffer);
}


// ESLAB

void NTM_Init()
{
	NTM_IRQ_INPUT_PU;    


    NTM_SCL_OUT;
    NTM_SDA_OUT;
	NTM_SDA_HIGH();  
	NTM_SCL_HIGH();	
	
	
	//test
		//NTM_SDA_IN;
}



//ESLAB
int NTM_Write(uint8_t* wrBuf, uint8_t wrLen){

	uint8_t i;
	uint8_t ack;
	
	ack = _i2c_Send_Byte(NTM_I2C_ADDR);
	for(i=0; i<wrLen; i++)
	{
		ack = _i2c_Send_Byte(wrBuf[i]);
	}
    stop_i2c();
    
	return ack; //0 is OK	

}



// ESLAB

int NTM_Read(uint8_t* rdBuf, uint8_t rdLen){

	uint8_t i;
	uint8_t ack;
	
	start_i2c();
    ack = _i2c_Send_Byte(NTM_I2C_ADDR | 0x01);
    for(i=0; i<rdLen; i++)
    {
        rdBuf[i]=_i2c_Read_Byte();
        _i2c_SendAck();
    }
    rdBuf[rdLen-1]=_i2c_Read_Byte();
    _i2c_SendNoack();
    stop_i2c();
    
    return ack;
}


//lgc 0724
uint8_t NTM_ReadRandom(uint8_t* rdBuf)
{
	uint8_t i,rdLen;
	uint8_t sum;
	start_i2c();
	_i2c_Send_Byte(NTM_I2C_ADDR | 0x01);
	rdBuf[0]=_i2c_Read_Byte();
	_i2c_SendAck();
	rdBuf[1]=_i2c_Read_Byte();
	_i2c_SendAck();
	rdLen =  rdBuf[1];      
	for(i=2; i<rdLen+2; i++)
	{
			rdBuf[i]=_i2c_Read_Byte();
			_i2c_SendAck();
	}
	rdBuf[rdLen+2]=_i2c_Read_Byte();
	_i2c_SendNoack();
	stop_i2c();
		
	sum=0;
	for(int i =0; i<rdLen+2;i++)
	{
		sum+=rdBuf[i];
	}
	uint8_t result = sum&0xFF;
	if(result == rdBuf[rdLen+2])
		return 1; //校验成功
	else
		return 0;// 校验失败				
		
}


unsigned char MAG3110_RandomRead(unsigned char RomAddress)
{
	unsigned char i = 0;
	start_i2c();
	i = i2c_send(IIC_WRITE_DEVICE_ADDRESS);
	if(i)
		return 0x0;
	i = i2c_send(RomAddress);
	if(i)
		return 0x0;
	delay_us(50); //延时50微秒
	start_i2c();
	i = i2c_send(IIC_READ_DEVICE_ADDRESS);
	if(i)
		return 0x0;
	i=i2c_read(0); //0：不发送ack
	stop_i2c();
	return(i);
}
	
uint8 MAG3110_ByteWrite(unsigned char RomAddress,unsigned char Data)
{
  	uint8 i = 0x0;
	start_i2c();
	i = i2c_send(IIC_WRITE_DEVICE_ADDRESS);
	if(i)
		return 0x1;	
	i = i2c_send(RomAddress);
	if(i)
		return 0x1;
	i = i2c_send(Data);
	if(i)
		return 0x1;
	stop_i2c();
	return 0x0;
}

/*********************************************************\
* Put MAG3110Q into Standby Mode
\*********************************************************/
void MAG3110_Standby (void)
{
  uint8 n;

  n = MAG3110_RandomRead( CTRL_REG1);
  MAG3110_ByteWrite(CTRL_REG1, n&0xFC|STANDBY_MASK);
}


void MAG3110_Init (void)     //用于触发测量模式 
{ 
  //P1SEL &= ~((0x1 << 2) | (0x1 << 3));
  //P1DIR |= ((0x1 << 2) | (0x1 << 3));
  delay_us(50);  
  MAG3110_Standby();   
  MAG3110_ByteWrite(CTRL_REG2,0x80);  //auto reset enable
  MAG3110_ByteWrite(CTRL_REG1,0x1A);  //启动测量 触发测量模式 
}

void MAG3110_Config(void)
{
	volatile unsigned char CTRL_REG2_temp=0;
	volatile unsigned char CTRL_REG1_temp=0;
	volatile unsigned char Current_Mode=0;
	
	MAG3110_ByteWrite(0x11, 0x80);
	CTRL_REG2_temp=MAG3110_RandomRead(0x11);
	MAG3110_ByteWrite(0x10, 0xC9);
	CTRL_REG1_temp=MAG3110_RandomRead(0x10);
	Current_Mode=MAG3110_RandomRead(0x08);
}


#if 0
MAG3110_Init();
HAL_Delay(500);

i= MAG3110_RandomRead(WHO_AM_I_REG);
if (i == MAG3110Q_ID)				//确认初始化是否成功
{
	while(1)
	{	
			//*********************  触发测量模式 ********************************************
		MAG3110_ByteWrite(CTRL_REG1,0x1A);	//启动测量 触发测量模式 
		i=MAG3110_RandomRead(STATUS_00_REG); 
		if(i&ZYXDR_MASK) //数据就绪
		{	
			DataX_H = MAG3110_RandomRead(OUT_X_MSB_REG); //读取X轴高字节
			DataX_L = MAG3110_RandomRead(OUT_X_LSB_REG); //读取X轴低字节
			DataY_H = MAG3110_RandomRead(OUT_Y_MSB_REG); //读取Y轴高字节
			DataY_L = MAG3110_RandomRead(OUT_Y_LSB_REG); //读取Y轴低字节
			DataZ_H = MAG3110_RandomRead(OUT_Z_MSB_REG); //读取Z轴高字节
			DataZ_L = MAG3110_RandomRead(OUT_Z_LSB_REG); //读取Z轴低字节

			x = ((DataX_H << 8) |DataX_L);
			y = ((DataY_H << 8) |DataY_L);
			z = ((DataZ_H << 8) |DataZ_L);
			x = y = z = 0;
		}
	}	
}
#endif


#if 0

/**************************
测量光张强度

***************************/

unsigned short get_light(void)
{       
        //unsigned char ack1=1;
        //unsigned char ack2=1;
        unsigned char ack3=1;
        unsigned char ack4=1;
        unsigned char ack5=1;
        unsigned char ack6=1;
        unsigned char ack7=1;
        //unsigned char ack8=1;
       
        unsigned char t0;
        unsigned char t1;
        unsigned short t;
        //int i;
        P1SEL &= ~((0x1 << 2) | (0x1 << 3));
        P1DIR |= ((0x1 << 2) | (0x1 << 3));
        P1DIR |= BV(3); //把p1_3设为输出功能 
        delay_us(200);
     

        start_i2c(); //发出 start 信号
        ack3 = i2c_send(0x1c);  //发出 从机地址 + 写操作
        if(ack3)                 //如果没有接收到来自从机的ack，则说明出错
           return 0xfffd;
        ack4 = i2c_send(0x01);// power on  发出通电命令
        if(ack4)
           return 0xfffc;
        stop_i2c();//BH1750FVI 不能在停机状态接收指令，所以要在每个 opecode(操作码) 后插入一个 stop 信号
                    //实测不发出上电指令也可以正确读出光照度
	
        start_i2c();
        ack5 = i2c_send(0x46);
        if(ack5)
           return 0xfffb;
        ack6 = i2c_send(0x10);// H- resolution mode  发出H分辨率测量命令   BH1750FVI 在一次测量中，结束状态转换为断电模式如果需要更新数据请 重新发送测量指令
        if(ack6)
           return 0xfffa;
        stop_i2c();
                       
       
        delay_us(1500); //等待bh1750测量好 最大等待时间 180ms
		
        start_i2c();
        ack7=i2c_send(0x47);   //发出 从机地址 + 读操作
        if(ack7)
           return 0xfff9;
                       
        t0=i2c_read(1); //读光照度的高8位，1标示发送ack信号
        t1=i2c_read(0); //读光照度的低8位，0标示不发送ack信号，光照数据都读完了，所以不用回ack
        stop_i2c();
        t=(((t0<<8)|t1)/1.2);


        return t;
       
}
#endif

