#include "bochiot_protocol.h"
#include <string.h>
#include <stdlib.h>



Func_callBack func_send_msg=NULL;

extern UART_HandleTypeDef		UartHandle1;
extern unsigned int CRC_CalCrc(char *pbuf, unsigned int size, unsigned int crc_val);
extern HAL_StatusTypeDef FLASH_Write(uint32_t Address, uint32_t *pData,uint32_t Data_len);
extern uint32_t FLASH_Read(uint32_t Address,uint32_t Read_len,uint32_t *pData,int FlashFlag);
extern void LORA_RXLED_On(void);
extern void bochiot_Electric_ctrl(char);
extern void bochiot_lock_ctrl(void);
extern uint8_t GetVoltageValue(void);
extern void del_log(uint8_t *gw_time2);
extern int write_card(void);
//ESLAB CODE
extern flash_password_t  g_local_password[MAX_CARD_NUM];
volatile static char index = 0;
volatile static char mac_number=0;
extern void bochiot_send_msg(uint8 *msg, uint8 msg_len);
extern void refresh_password_status(void);//lgc0725
extern void del_card_id(uint8_t *del_uid);//lgc0803
extern void RTC_TimeStampConfig(uint8_t* time);

//ESLAB CODE END

extern ENDEVICE_INFO_t g_endevice_info;
extern uint8_t NodeSixByteMac[8];

static int __anaylyze_payload(uint8* gw_addr, uint8* frame_buf);
int __ctrl_dev(void);
static int __heart_beat(void);
static int __register_request(uint8 * gw_addr);
static int __get_voltage(void);


//取电开关控制
volatile static char switch_ctrol = 0;
//控制命令类型
volatile static char Cmd_Type = 0;




uint8 bochiot_check_str(uint8 *retPacket,uint8 len)
{
	if(retPacket==NULL)
		return 0;

	uint8 check_result=retPacket[0];

	for(uint8 i=1;i<len;i++)
	{
		check_result ^= retPacket[i];
	
	}
	return check_result;
}

//if get successful , return 0 , else return -1;
int bochiot_get_gwAddr(uint8 * ret_Addr)
{
	if(ret_Addr==NULL) return -1;
	//return 0;
#if 0
	ret_Addr[0] = 0x1a;
	ret_Addr[1] = 0xaa;
	ret_Addr[2] = 0x01;
	ret_Addr[3] = 0x00;
	ret_Addr[4] = 0x00;
	ret_Addr[5] = GwMac;
	return 0;
#endif
#if 1
//	memset(&g_endevice_info,0,sizeof(g_endevice_info));
//	FLASH_Read(FLASH_USER_START_ADDR,sizeof(g_endevice_info),(uint32_t*)&g_endevice_info,0);
	memcpy(ret_Addr,g_endevice_info.server_addr,MAC_GW_LEN);
	return 0;
#endif
}

//if set successful , return 0 , else return -1;
int bochiot_set_gwAddr(uint8 * set_Addr)
{
	if(set_Addr==NULL) return -1;

#if 1
	memcpy(g_endevice_info.server_addr,set_Addr,MAC_GW_LEN);
	g_endevice_info.regStatus = 1;
	if(FLASH_Write(FLASH_MAC_START_ADDR,(uint32_t*)&g_endevice_info,sizeof(g_endevice_info))!=HAL_OK)
	{
		return -1;
	}
#endif	
	return 0;
}

//if set successful , return 0 , else return -1;
int bochiot_set_localAddr(uint8 * set_Addr)
{
	if(set_Addr==NULL) return -1;

	memset(&g_endevice_info,0,sizeof(g_endevice_info));
	FLASH_Read(FLASH_MAC_START_ADDR,sizeof(g_endevice_info),(uint32_t*)&g_endevice_info,0);
	memcpy(g_endevice_info.local_addr,set_Addr,MAC_LEN);
	if(FLASH_Write(FLASH_MAC_START_ADDR,(uint32_t*)&g_endevice_info,sizeof(g_endevice_info))!=HAL_OK)
	{
		return -1;
	}
	
	return 0;
}


//if get successful , return 0 , else return -1;


int bochiot_get_localAddr(uint8 * ret_Addr, int Mode)
{
	if(ret_Addr==NULL) return -1;
#if 0
//			ret_Addr[0] = 0x1A;
//			ret_Addr[1] = 0xAA;
			ret_Addr[0] = DeviceType;
			ret_Addr[1] = 0x00;
			ret_Addr[2] = 0x00;
			ret_Addr[3] = RoomNumber;
			return 0;
#endif
#if 0
			ret_Addr[0] = 0xAA;
			ret_Addr[1] = 0xAA;
			ret_Addr[2] = 0xAA;
			ret_Addr[3] = 0xAA;
			ret_Addr[4] = 0xAA;
			ret_Addr[5] = 0xA0;
			return 0;
#endif
	
	if(Mode == RcvPack)
	{
		memcpy(ret_Addr,g_endevice_info.local_addr,MAC_LEN);
	}
	else if(Mode == BackPack)
	{
		memcpy(ret_Addr,(char*)NodeSixByteMac,MAC_LEN+2);
	}
	return 0;

}

//return the type of the node
uint8 bochiot_get_dev_type(void)
{	
#if 1
	return 1;
#endif
#if 0
	memset(&g_endevice_info,0,sizeof(g_endevice_info));
	FLASH_Read(FLASH_USER_START_ADDR,sizeof(g_endevice_info),(uint32_t*)&g_endevice_info,0);
	return g_endevice_info.node_type;
#endif
}

uint8 bochiot_get_parkSpot_status(void)
{
#if 1
		return 1;
#endif
#if 0
	memset(&g_endevice_info,0,sizeof(g_endevice_info));
	FLASH_Read(FLASH_USER_START_ADDR,sizeof(g_endevice_info),(uint32_t*)&g_endevice_info,0);
	return g_endevice_info.status;
#endif
}

uint8 bochiot_get_devSwitch_status(void)
{
#if 1
		return 0;
#endif
#if 0
	memset(&g_endevice_info,0,sizeof(g_endevice_info));
	FLASH_Read(FLASH_USER_START_ADDR,sizeof(g_endevice_info),(uint32_t*)&g_endevice_info,0);
	return g_endevice_info.regStatus;
#endif
}

int bochiot_GetMacConfigStatus(void)
{
	memset(&g_endevice_info,0,sizeof(g_endevice_info));
	FLASH_Read(FLASH_MAC_START_ADDR,sizeof(g_endevice_info),(uint32_t*)&g_endevice_info,0);
	if(g_endevice_info.ConfigFlag[0]==0xaa && g_endevice_info.ConfigFlag[0]==0x55) return 1;
	return 0;
}

int bochiot_SetMacConfig(char* mac)
{

	memset(&g_endevice_info,0,sizeof(g_endevice_info));
	//FLASH_Read(FLASH_USER_START_ADDR,sizeof(g_endevice_info),(uint32_t*)&g_endevice_info,0);
	g_endevice_info.ConfigFlag[0] = 0xaa;
	g_endevice_info.ConfigFlag[1] = 0x55;
	memcpy(g_endevice_info.local_addr,mac,MAC_LEN);
	if(FLASH_Write(FLASH_MAC_START_ADDR,(uint32_t*)&g_endevice_info,sizeof(g_endevice_info))!=HAL_OK)
	{
		return -1;
	}
	return 0;
}


/*
function_name: bochiot_protocol_init
function 	 : you must call this function first if you want to you this bochiot_protocol.c
parameters   : send_msg , call back func 
return 		 : null
*/
void bochiot_protocol_init(Func_callBack send_msg)
{
	func_send_msg = send_msg;
}

#if 1
/*
function_name: bochiot_packet_payload
function 	 : Packaged load
parameters   : pData, type(MAC_PAYLOAD_DATA *), the payload data
			   retPakcet, type(uint8*), the result str of Packaged load
return 		 : the len of the Packaged, type(int), if return -1, packaged wrong
*/
int bochiot_packet_payload(MAC_PAYLOAD_DATA *payload, uint8 *retPacket)
{
	if(payload==NULL || retPacket==NULL) return -1;
	
	MAC_FRAME_HEAD frame_head;
	//uint32 crc_value = 0xFFFFFFFF;
	uint8 check_result = 0;
	int len=0;

	memset(&frame_head,0,sizeof(frame_head));
	frame_head.protocol_type = PROTOCOL_TYPE;
	//dst_adr_len+src_adr_len+node_type_len+(cmd_type+payload_data_len+strlen(pData))+crc
	frame_head.frame_data_len[1] = MAC_GW_LEN*2+SIZE_node_type+SIZE_protocol+(sizeof(MAC_PAYLOAD_DATA)-1+payload->payload_data_len)+SIZE_crc;
	frame_head.frame_data_len[0] = (MAC_GW_LEN*2+SIZE_node_type+SIZE_protocol+(sizeof(MAC_PAYLOAD_DATA)-1+payload->payload_data_len)+SIZE_crc)>>8;
	if(bochiot_get_gwAddr(frame_head.dst_adr)==-1) return -1;
	if(bochiot_get_localAddr(frame_head.src_adr, BackPack)==-1) return -1;
	frame_head.node_type = node_gateway;

	memcpy(retPacket,&frame_head,sizeof(frame_head));
	len += sizeof(frame_head);
	memcpy(retPacket+len,payload,sizeof(MAC_PAYLOAD_DATA)-1);
	len += (sizeof(MAC_PAYLOAD_DATA)-1);
	memcpy(retPacket+len,payload->pData,payload->payload_data_len);
	len += payload->payload_data_len;

	check_result = bochiot_check_str(retPacket,len);
	retPacket[len++] = check_result;
	retPacket[len++] = 0xFF;

	return len;
}
#endif
/*
int check_mac(const char *mac,char *buffer)
{
	int len=0,count=0,skip=0; 
	uint8  get_cmd_type=0;
//	uint8 mac_number;
	mac_number = buffer[0];
	if(mac_number > 5) return 1;
	for(count=0;count<mac_number;count++)
	{
		if(memcmp(mac, buffer+skip+1,4)==0)
		{
			get_cmd_type = mac_number*4+7+count*2;
			Cmd_Type = buffer[get_cmd_type];

			if(buffer[skip+1] == Electrical_Switch)
			{	
				get_cmd_type+=1;
				switch_ctrol = buffer[get_cmd_type];
			}
			return 0;	
		}
		skip+=4;
	}
	return 1;
}

*/

//lgc 0803 重写以正确接收包含多条控制命令的数据包
int check_mac(const char *mac,char *buffer)
{
	int count=0,skip=0; 
	uint8  get_cmd_type=0;
//	uint8 mac_number;
	mac_number = buffer[0];
	if(mac_number > 5) return 1;
	index=0;
	for(count=0;count<mac_number;count++)
	{
		get_cmd_type = mac_number*4+7+count*2;
		if(buffer[get_cmd_type]==cmd_add_card)
			index+=10;	
		else if(buffer[get_cmd_type]==cmd_del_log||buffer[get_cmd_type]==cmd_del_card||buffer[get_cmd_type]==cmd_synchronize_tim)
			index+=6;	
		if(memcmp(mac, buffer+skip+1,4)==0)
		{
			if(buffer[get_cmd_type]==cmd_add_card)
				index-=10;	
			else if(buffer[get_cmd_type]==cmd_del_log||buffer[get_cmd_type]==cmd_del_card||buffer[get_cmd_type]==cmd_synchronize_tim)
				index-=6;	
			Cmd_Type = buffer[get_cmd_type];
			//index = buffer[get_cmd_type+1];//van's code
			if(buffer[skip+1] == Electrical_Switch)
			{	
				get_cmd_type+=1;
				switch_ctrol = buffer[get_cmd_type];
			}
			return 0;	
		}
		skip+=4;
	}
	index=0;//调试
	return 1;
}


/*
function_name: bochiot_anaylyze_frame
function 	 : anaylyze recv frame msg
parameters   : frame_buf, type(uint8 *), recv frame buff
			   buf_len, type(uint8), the len of recv frame buff
return 		 : return 0, right pack, return -1, wrong pack
*/
int bochiot_anaylyze_frame(uint8 * frame_buf,uint8 buf_len)
{
	if(frame_buf==NULL) return -1;
	if(frame_buf[0]!=0xb0 || frame_buf[1]!=0xc0) return -1;
		
	//uint32 crc_value = 0xFFFFFFFF, crc_value_recv=0;
	uint16 protocol_type=0;
	uint8 local_addr[MAC_LEN],gw_addr[MAC_GW_LEN];
	uint8 Mac_number = 0;

	Mac_number = frame_buf[SIZE_proto_type+SIZE_data_len];
#if 0
	check_result = bochiot_check_str(frame_buf,buf_len-2);
	if(check_result!=frame_buf[buf_len-2] && frame_buf[buf_len-1]!=0xFF)
	{
		return -1;
	}
#endif

	memset(local_addr,0,MAC_LEN);
	memset(gw_addr,0,MAC_GW_LEN);
	if(bochiot_get_localAddr(local_addr, RcvPack)!=0) return -1;
#if 1
	if(check_mac((char*)local_addr,(char*)frame_buf+sizeof(MAC_FRAME_HEAD_RX)-1)!=0)
	{
		//this pack is not for me , ignore it;
		return -1;
	}
#endif
	//if(bochiot_get_gwAddr(gw_addr)!=0) return -1;
	memcpy(gw_addr,frame_buf+sizeof(MAC_FRAME_HEAD_RX)+Mac_number*MAC_LEN,MAC_GW_LEN);
		
	/* 判断控制中心是否已经注册 */
	if(memcmp((char*)gw_addr,(char*)g_endevice_info.server_addr,MAC_GW_LEN)!=0)
	{
		if(bochiot_set_gwAddr(gw_addr)!=0)
			return -1;
	}
	
	
#if 0
	/* 判断是否已经注册 */
	if(memcmp((char*)gw_addr,(char*)g_endevice_info.server_addr,MAC_GW_LEN)!=0)
	{
		if(bochiot_set_gwAddr(gw_addr)!=0)
			return -1;
	}
#endif
	
	memcpy(&protocol_type,frame_buf,SIZE_proto_type);
	if(protocol_type!=PROTOCOL_TYPE)
	{
		// wrong protocol type
		return -1;
	}
	
	return __anaylyze_payload(gw_addr , frame_buf);
}

//	uint8 card_uid[4];
//	uint8 expire_time[6];

static int __anaylyze_payload(uint8* gw_addr, uint8_t* frame_buf)
{
	uint8_t syc_time[6];
	uint8_t gw_time[6];
	int status = -1;
	uint8 add_card_uid[4];
	uint8 expire_time[6];
   
	switch(Cmd_Type)
	{
		case cmd_register:
			//gw_addr
			status = __register_request(gw_addr);
			Cmd_Type = 0;
			break;
		case cmd_heart_beat:
			status = __heart_beat();
			Cmd_Type = 0;
			break;
		case cmd_ctrl_msg:
			status = __ctrl_dev();
			Cmd_Type = 0;
			break;
		case cmd_get_voltage:
			status = __get_voltage();
			Cmd_Type = 0;
			break;
		
		
		// ESLAB CODE START
		case cmd_add_card: //下发卡号		
		memcpy(&add_card_uid,&frame_buf[sizeof(MAC_FRAME_HEAD_RX)+mac_number*MAC_LEN+MAC_GW_LEN+mac_number*2+index*10],4); //填入UID
		memcpy(&expire_time,&frame_buf[sizeof(MAC_FRAME_HEAD_RX)+mac_number*MAC_LEN+MAC_GW_LEN+mac_number*2+index*10]+4,6); //填入EXPIRE TIME
		
		add_card_handling(add_card_uid, expire_time,g_endevice_info.server_addr);		
		status = 0x06;

		break;
		
		case cmd_del_log:	 //下发删除日志操作

			memcpy(&gw_time,&frame_buf[sizeof(MAC_FRAME_HEAD_RX)+MAC_LEN+MAC_GW_LEN+1],6); 		
			del_open_log(gw_time);
		  status = 0x07;
			
		break;
		
		case cmd_synchronize_tim: //同步时间
			memcpy(&syc_time,&frame_buf[sizeof(MAC_FRAME_HEAD_RX)+MAC_LEN+MAC_GW_LEN+1],6);
		    RTC_TimeStampConfig(syc_time); // RTC??
			status = 0x08;
		break;
		
//		case cmd_del_card://删除卡号lgc0803
//			memcpy(&card_uid,&frame_buf[sizeof(MAC_FRAME_HEAD_RX)+mac_number*MAC_LEN+MAC_GW_LEN+mac_number*2+index],4); //填入UID
//			del_card_id(card_uid);
//			del_card_ask(g_endevice_info.server_addr,card_uid);
//			status = 0x0a;
//		break;
		
		
		default:
		break;	
		
	}
	return status;
}

static int __register_request(uint8 * gw_addr)
{
	if(gw_addr==NULL) return -1;
	MAC_PAYLOAD_DATA *payload=NULL;
	uint8 retPacket[64];
	int ret_len;

	char testGw[12];
	
	memset(testGw,0,12);
	memcpy(testGw,gw_addr,6);
#if 1
	/* 判断是否已经注册 */
	if(memcmp((char*)gw_addr,(char*)g_endevice_info.server_addr,MAC_GW_LEN)!=0)
	{
		if(bochiot_set_gwAddr(gw_addr)!=0)
			return -1;
	}
#endif

	//register ok,ack a msg
	volatile uint8 node_type=0;
	node_type = bochiot_get_dev_type();
	payload = (MAC_PAYLOAD_DATA*)malloc(sizeof(MAC_PAYLOAD_DATA)-1+1);
	memset(payload,0,sizeof(MAC_PAYLOAD_DATA)-1+1);

   //ctr device function , 
   // bochiot_get_devSwitch_status should after ctrl		
	payload->cmd_type = cmd_register;
	payload->payload_data_len = 0;

	memset(retPacket,0,64);
//	  	ret_len = bochiot_packet_payload(payload,retPacket);		
	free(payload);
	payload = NULL;
	if(ret_len==-1) return -1;

	if(func_send_msg==NULL) return -1;
	//func_send_msg("return massage!", strlen("return massage!"));
	
	
	func_send_msg(retPacket,ret_len);		
	extern void UARTx_Write(UART_HandleTypeDef *huart, char* Value, byte size);
	UARTx_Write(&UartHandle1, (char *)retPacket, ret_len);
	return 0;
}

static int __heart_beat(void)
{

	MAC_PAYLOAD_DATA *payload=NULL;
	uint8 retPacket[64],ret_len;

	//ack_result(1byte)+device_info(device_type(1byte)+spot_status(1byte)+status(1byte));
	payload = (MAC_PAYLOAD_DATA*)malloc(sizeof(MAC_PAYLOAD_DATA)-1+1+3);
	memset(payload,0,sizeof(MAC_PAYLOAD_DATA)-1+1+3);
	payload->cmd_type = cmd_heart_beat;
	payload->payload_data_len = 4;

	payload->pData[0] = 0x01;
	payload->pData[1] = bochiot_get_dev_type();
	payload->pData[2] = bochiot_get_parkSpot_status();
	payload->pData[3] = bochiot_get_devSwitch_status();
	
//	ret_len = bochiot_packet_payload(payload,retPacket);

	if(func_send_msg==NULL) return -1;
	func_send_msg(retPacket,ret_len);
	free(payload);
	payload = NULL;
	return 2;
}

int __ctrl_dev(void)
{	
	
	#if NODE_TYPE
		bochiot_lock_ctrl();
	#else	
		bochiot_Electric_ctrl(switch_ctrol);	
	#endif
	 
	return 2;
}

static int __get_voltage(void)
{
#if 0
	MAC_PAYLOAD_DATA *payload=NULL;
	uint8 retPacket[64];
	int ret_len;

		volatile uint8 node_type=0;
		node_type = bochiot_get_dev_type();
		payload = (MAC_PAYLOAD_DATA*)malloc(sizeof(MAC_PAYLOAD_DATA)-1+1);
		memset(payload,0,sizeof(MAC_PAYLOAD_DATA)-1+1);

	   //ctr device function , 
	   // bochiot_get_devSwitch_status should after ctrl		
		payload->cmd_type = cmd_get_voltage;
		payload->payload_data_len = 1;
		payload->pData[0] = GetVoltageValue();
		memset(retPacket,0,64);
	  ret_len = bochiot_packet_payload(payload,retPacket);		
		free(payload);
		payload = NULL;
		if(ret_len==-1) return -1;

		if(func_send_msg==NULL) return -1;
		//func_send_msg("return massage!", strlen("return massage!"));
		
		
		func_send_msg(retPacket,ret_len);		
//		extern void UARTx_Write(UART_HandleTypeDef *huart, char* Value, byte size);
//		UARTx_Write(&UartHandle1, (char *)retPacket, ret_len);	
#else
	
	MAC_PAYLOAD_DATA *payload=NULL;
	uint8 retPacket[256];
	int ret_len;

	payload = (MAC_PAYLOAD_DATA*)malloc(sizeof(MAC_PAYLOAD_DATA)-1+3);
	memset(payload,0,sizeof(MAC_PAYLOAD_DATA)-1+3);

    //ctr device function , 
    // bochiot_get_devSwitch_status should after ctrl
    #if 1
	uint8 node_type=0;
	node_type = bochiot_get_dev_type();
	
	payload->cmd_type = cmd_report_msg;
	payload->payload_data_len = 3;
	payload->pData[0] = node_type;
	payload->pData[1] = 1;   //lock status, ignore
	payload->pData[2] = GetVoltageValue();   //
	
	//memcpy(retPacket,payload,6);
	#endif

	memset(retPacket,0,256);
	ret_len = bochiot_packet_payload(payload,retPacket);
	free(payload);
	payload = NULL;
	if(ret_len==-1) return -1;
	if(func_send_msg==NULL) return -1;
	func_send_msg(retPacket,ret_len);
	return 0;
#endif
}


int MAG3110_XOFF=0,MAG3110_YOFF=0;
int MAG3110_XMax,MAG3110_YMax,MAG3110_XMin,MAG3110_YMin;
int  mag3110_adjust_position(short *data_x,short *data_y,short *data_z)
{
	static int first_flag = 1;
	if (first_flag)
	{
		MAG3110_XMax = *data_x;
		MAG3110_XMin = *data_x;
		MAG3110_YMax = *data_y;
		MAG3110_YMin = *data_y;
		first_flag = 0;
	}
	if (*data_x > MAG3110_XMax)
	{
		MAG3110_XMax =  *data_x;
	}
	else if (*data_x < MAG3110_XMin)
	{
		MAG3110_XMin =  *data_x;
	}
	if (*data_y > MAG3110_YMax)
	{
		MAG3110_YMax =  *data_y;
	}
	else if (*data_y < MAG3110_YMin)
	{
		MAG3110_YMin =  *data_y;
	}
	MAG3110_XOFF = (MAG3110_XMax + MAG3110_XMin) / 2;
	MAG3110_YOFF = (MAG3110_YMax + MAG3110_YMin) / 2;
	
	return 1;
}


uint8 bochiot_read_spot_status(void)
{
#if	1
	/*
	geomagnetism read i2c
	*/
	int i=0,count=SENSOR_XYZ_COUNT;
	uint8 DataX_H=0,DataX_L=0,DataY_H=0,DataY_L=0,DataZ_H=0,DataZ_L=0;
	short x=0,y=0,z=0;
	int x_add=0,y_add=0,z_add=0;
	MAG3110_Init();
	HAL_Delay(100);
	i= MAG3110_RandomRead(WHO_AM_I_REG);
	MAG3110_Config();
	if (i == MAG3110Q_ID)				//确认初始化是否成功
	{
		//while(count>0)
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
				mag3110_adjust_position(&x,&y,&z);
				x_add += x;
				y_add += y;
				z_add += z;
				DB_int(x,"xyz="," ");
				DB_int(y,""," ");
				DB_int(z,"","\r\n");
				x = y = z = 0;
				count--;
			}
			HAL_Delay(20);
		}	
	}

	
	z_add = z_add/SENSOR_XYZ_COUNT;
	//DB_int(z_add,"z_add1=","\r\n");
	//z_add = z_add-COMMON_VALUE;
	//DB_int(z_add,"z_add2=","\r\n");
	if(z_add<0x100)
	{
		//DB_STR("sensor set 1\r\n");
		return 0;
	}
#endif
	return 1;
}

int bochiot_report_status(int flag)
{
	static uint8 spot_status = 0;
	uint8 current_status = 0;

	current_status= bochiot_read_spot_status();
	//DB_int(current_status,"current=","\r\n");
	//DB_int(spot_status,"spot_status=","\r\n");
	if(flag==1)
	{
		if(spot_status==current_status)
		{
			return -1;
		}
	}
	spot_status = current_status;
	
	MAC_PAYLOAD_DATA *payload=NULL;
	uint8 retPacket[64];
	int ret_len;

	//ack_result(1byte)+device_info(device_type(1byte)+spot_status(1byte)+status(1byte));
	payload = (MAC_PAYLOAD_DATA*)malloc(sizeof(MAC_PAYLOAD_DATA)-1+1+3);
	memset(payload,0,sizeof(MAC_PAYLOAD_DATA)-1+1+3);

    //ctr device function , 
    // bochiot_get_devSwitch_status should after ctrl
    #if 1
	//这里如果不使用临时变量(node_type,reg_status),则当调用bochiot_get_dev_type===的时候会出错，不知道原因
	uint8 node_type=0,reg_status=0;
	node_type = bochiot_get_dev_type();
	reg_status = bochiot_get_devSwitch_status();		
	payload->cmd_type = cmd_report_msg;
	payload->payload_data_len = 4;
	payload->pData[0] = 0x01;
	payload->pData[1] = node_type;
	payload->pData[2] = spot_status;
	payload->pData[3] = reg_status;
	//memcpy(retPacket,payload,6);
	#endif

	memset(retPacket,0,64);
//	ret_len = bochiot_packet_payload(payload,retPacket);
	if(ret_len==-1) return -1;
	//DB_HEX_str("-len=",(uint8*)&ret_len,4);
	//DB_HEX_str("ret=",retPacket,ret_len);
	//UARTx_Write(&UartHandle1,"hello\r\n",strlen("hello\r\n"));
	//bochiot_anaylyze_frame(retPacket,ret_len);

	if(func_send_msg==NULL) return -1;
	func_send_msg(retPacket,ret_len);
	free(payload);
	payload = NULL;
	return 0;
}

void bochiot_request_register(void)
{	
	MAC_PAYLOAD_DATA *payload=NULL;
	uint8 retPacket[64];
	int ret_len;

	//ack_result(1byte)+device_info(device_type(1byte)+spot_status(1byte)+status(1byte));
	payload = (MAC_PAYLOAD_DATA*)malloc(sizeof(MAC_PAYLOAD_DATA)-1+1+3);
	memset(payload,0,sizeof(MAC_PAYLOAD_DATA)-1+1+3);

	//ctr device function , 
	// bochiot_get_devSwitch_status should after ctrl
#if 1
	//这里如果不使用临时变量(node_type,reg_status),则当调用bochiot_get_dev_type===的时候会出错，不知道原因
	payload->cmd_type = cmd_register;
	payload->payload_data_len = 0;
	//memcpy(retPacket,payload,6);
#endif

	memset(retPacket,0,64);
//	ret_len = bochiot_packet_payload(payload,retPacket);
	if(ret_len==-1) return;
	//DB_HEX_str("-len=",(uint8*)&ret_len,4);

	if(func_send_msg==NULL) return ;
	func_send_msg(retPacket,ret_len);

	free(payload);
	payload = NULL;
}

//if have been registered, return 1 , else return 0;
int bochiot_check_register(void)
{
	memset(&g_endevice_info,0,sizeof(g_endevice_info));
	FLASH_Read(FLASH_MAC_START_ADDR,sizeof(g_endevice_info),(uint32_t*)&g_endevice_info,0);
	return g_endevice_info.regStatus;
}

int bochiot_heart_beat(void)
{
	uint8 spot_status = 0;

	spot_status= bochiot_read_spot_status();
	
	MAC_PAYLOAD_DATA *payload=NULL;
	uint8 retPacket[64];
	int ret_len;

	//ack_result(1byte)+device_info(device_type(1byte)+spot_status(1byte)+status(1byte));
	payload = (MAC_PAYLOAD_DATA*)malloc(sizeof(MAC_PAYLOAD_DATA)-1+1+3);
	memset(payload,0,sizeof(MAC_PAYLOAD_DATA)-1+1+3);

    //ctr device function , 
    // bochiot_get_devSwitch_status should after ctrl
    #if 1
	//这里如果不使用临时变量(node_type,reg_status),则当调用bochiot_get_dev_type===的时候会出错，不知道原因
	uint8 node_type=0,reg_status=0;
	node_type = bochiot_get_dev_type();
	reg_status = bochiot_get_devSwitch_status();		
	payload->cmd_type = cmd_report_msg; //现在演示版使用cmd_report_msg,正式版会修改为cmd_heart_beat
	payload->payload_data_len = 4;
	payload->pData[0] = 0x01;
	payload->pData[1] = node_type;
	payload->pData[2] = spot_status;
	payload->pData[3] = reg_status;
	//memcpy(retPacket,payload,6);
	#endif

	memset(retPacket,0,64);
//	ret_len = bochiot_packet_payload(payload,retPacket);
	if(ret_len==-1) return -1;
	//DB_HEX_str("-len=",(uint8*)&ret_len,4);
	//DB_HEX_str("ret=",retPacket,ret_len);
	//UARTx_Write(&UartHandle1,"hello\r\n",strlen("hello\r\n"));
	//bochiot_anaylyze_frame(retPacket,ret_len);

	if(func_send_msg==NULL) return -1;
	func_send_msg(retPacket,ret_len);
	free(payload);
	payload = NULL;
	return 0;
}

