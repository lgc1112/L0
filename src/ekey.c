#include "ekey.h"

//电子钥匙开门功能所需结构体
static __align(4) flash_queue_t 		g_FIFO_queue;
static __align(4) flash_password_t	g_local_password[MAX_CARD_NUM];
static __align(4) local_log_t 		g_open_log[MAX_LOG_NUM];

//重传计数器
static uint8_t g_resend_count = 0;

//--------------------------------------------------------------------------------------------
//  Section 刷卡处理部分: 刷卡处理API
//--------------------------------------------------------------------------------------------
void swipe_card_handling(uint8_t* mes)
{
	uint8_t card_id[4];
	
	get_opencard_id(card_id, mes);
	
	if(judge_pwd_valid(card_id) == 1){
		open_lock();
		add_open_log(card_id, 1);
	}else{
		add_open_log(card_id, 0);
	}
}

//--------------------------------------------------------------------------------------------
//  Section 刷卡处理部分: 获取刷卡卡号
//--------------------------------------------------------------------------------------------
static void get_opencard_id(uint8_t *id, uint8_t* mes)
{
	if(mes[0] == RSP_I2C_MSG_CARD_A) //card A
		memcpy(id, &mes[2], 4);
	else                             //card B
		memcpy(id, &mes[8], 4);	
}

//--------------------------------------------------------------------------------------------
//  Section 刷卡处理部分: 判断刷卡是否有效
//--------------------------------------------------------------------------------------------
static uint8_t judge_pwd_valid(uint8_t *id)
{
	refresh_password_status();
	
	for(int i =0; i < MAX_CARD_NUM; i++){
		if(memcmp(id, g_local_password[i].uid, 4) == 0 && g_local_password[i].flag[0] == 1) 	
			return 1;
	}	
	return 0;	
} 

//--------------------------------------------------------------------------------------------
//  Section 刷卡处理部分: 增加日志、写入flash
//--------------------------------------------------------------------------------------------
 static void add_open_log(uint8_t* uid,int flag)
 {
	uint8_t time[6];
	 
	if( g_FIFO_queue.count == MAX_LOG_NUM )
		return;
	
	get_real_tim(time);
	
	memcpy(&g_open_log[g_FIFO_queue.head].uid,uid,4);
	g_open_log[g_FIFO_queue.head].year = time[0];
	g_open_log[g_FIFO_queue.head].month = time[1];
	g_open_log[g_FIFO_queue.head].day = time[2];
	g_open_log[g_FIFO_queue.head].hour = time[3];
	g_open_log[g_FIFO_queue.head].min = time[4];
	g_open_log[g_FIFO_queue.head].sec = time[5];
	g_open_log[g_FIFO_queue.head].ifopen = flag;
	
	enqueue_open_log();
	FLASH_Write(FLASH_LOG_START_ADDR, (uint32_t*)&g_open_log, sizeof(g_open_log));
	
	g_resend_count = 0;	 
 }

//--------------------------------------------------------------------------------------------
//  Section 刷卡处理部分: 开门日志入队
//--------------------------------------------------------------------------------------------
static void enqueue_open_log()
{	
	g_FIFO_queue.count++;
	g_FIFO_queue.head++;
	
	if(g_FIFO_queue.head >= MAX_LOG_NUM)
		g_FIFO_queue.head = 0;	

	FLASH_Write(FLASH_QUEUE_START_ADDR, (uint32_t*)&g_FIFO_queue, sizeof(g_FIFO_queue));//把queue 存进flash
}


 
//--------------------------------------------------------------------------------------------
//  Section 发卡处理部分: 下发卡号处理API
//--------------------------------------------------------------------------------------------
void add_card_handling(uint8 *add_card_uid, uint8 *expire_time, uint8_t *gwmac)
{
	int flag = 0;	
	
	flag = get_write_card_idx(add_card_uid);
	if(get_write_card_idx(add_card_uid) == -1)
		return;
	
	memcpy(&g_local_password[flag].uid, add_card_uid, 4); //填入UID
	memcpy(&g_local_password[flag].expire_time, expire_time, 6); //填入EXPIRE TIME	
	g_local_password[flag].flag[0] = 1;
	
	refresh_password_status();
	
	FLASH_Write(((uint32_t)ADDR_FLASH_PASSWORD_PAGE1), (uint32_t*)&g_local_password, sizeof(g_local_password));  //写入flash			
	
	add_card_ask(gwmac, g_local_password[flag].uid); // send ack;			
}

//--------------------------------------------------------------------------------------------
//  Section 发卡处理部分: 发送发卡接收反馈ack
//--------------------------------------------------------------------------------------------
static void add_card_ask(uint8_t *gwmac, uint8_t *uid)
{
	uint8_t send_msg[29];
	mes_to_updata_t mes_to_gateway;
	
	mes_to_gateway.protocol_type[0] = 0xb0;
	mes_to_gateway.protocol_type[1] = 0xc0;
	mes_to_gateway.datalength[0] = 0x00; //36bytes
	mes_to_gateway.datalength[1] = 0x19;//sizeof(g_mes_to_gateway)-4;
	mes_to_gateway.nodemac[0] = 0x1a;
	mes_to_gateway.nodemac[1] = 0xaa;	 
	mes_to_gateway.nodemac[2] = DeviceType;
	mes_to_gateway.nodemac[3] = DevcMAC4;
	mes_to_gateway.nodemac[4] = DevcMAC5;
	mes_to_gateway.nodemac[5] = DevcMAC6;
	mes_to_gateway.cmd = cmd_add_card;
	mes_to_gateway.payloadlength = 0x07; //7bytes
	mes_to_gateway.device_type = 0x02; //
	mes_to_gateway.protocol_ID[0] = 0x11;
	mes_to_gateway.protocol_ID[1] = 0x22;

	memcpy(mes_to_gateway.uid, uid, 4);
	memcpy(mes_to_gateway.gwmac, gwmac, 6);	
	memcpy(&send_msg, &mes_to_gateway, 27);//除了checksum和0xff都复制过来;
	
	send_msg[27] = bochiot_check_str(send_msg, sizeof(send_msg)-2);
	send_msg[28] = 0xFF;
	
	bochiot_send_msg((uint8*)&send_msg, 29);
}

//--------------------------------------------------------------------------------------------
//  Section发卡处理部分: 获取可写入卡号的位置
//--------------------------------------------------------------------------------------------
static int get_write_card_idx(uint8_t* recv_uid)
{
	for(int i=0; i< MAX_CARD_NUM; i++){  //判断id是否存在，是则返回改id位置
		if((memcmp(recv_uid, g_local_password[i].uid, 4) == 0)) 
			return i;
	}
	
	refresh_password_status();
	
	for(int i=0; i< MAX_CARD_NUM; i++){  //新id，寻找空位置存入
		if(g_local_password[i].flag[0] == 0)
			return i;	
	}
	
	return -1;
}



//--------------------------------------------------------------------------------------------
//  Section 删除日志部分: 删除日志处理API
//--------------------------------------------------------------------------------------------
void del_open_log(uint8_t *gw_time2)
{	
	if(g_FIFO_queue.count == 0)
		return;
	if(memcmp(&g_open_log[g_FIFO_queue.tail].year, gw_time2, 6) != 0)
		return;

	memset(&g_open_log[g_FIFO_queue.tail], 0, sizeof(g_open_log[g_FIFO_queue.tail]));
	
	dequeue_open_log();	
	
	FLASH_Write(FLASH_LOG_START_ADDR, (uint32_t*)&g_open_log, sizeof(g_open_log));
	
	g_resend_count = 0;
}

//--------------------------------------------------------------------------------------------
//  Section 删除日志部分: 开门日志出队
//--------------------------------------------------------------------------------------------
static void dequeue_open_log(void)
{
	g_FIFO_queue.count--;
	g_FIFO_queue.tail++;
	
	if(g_FIFO_queue.tail >= MAX_LOG_NUM)
		g_FIFO_queue.tail = 0;	
	
	FLASH_Write(FLASH_QUEUE_START_ADDR, (uint32_t*)&g_FIFO_queue, sizeof(g_FIFO_queue));//把queue 存进flash
}
 
 
//--------------------------------------------------------------------------------------------
//  刷新卡号控制位flag
//--------------------------------------------------------------------------------------------
void refresh_password_status(void)
{
	uint8_t time[6];
	
	get_real_tim(time);
	
	for(int i = 0; i < MAX_CARD_NUM; i++){
		if(g_local_password[i].flag[0] == 0)
			continue;
		
		if(memcmp(&time, &g_local_password[i].expire_time, 6) > 0)	
			g_local_password[i].flag[0] = 0;					
	}

}


//--------------------------------------------------------------------------------------------
//   初始化flash
//--------------------------------------------------------------------------------------------
void ekey_flash_init(void)
{
//    memset(g_local_password, 0, sizeof(g_local_password));  //lgc0728 erase the flash
//	  FLASH_Write(((uint32_t)ADDR_FLASH_PASSWORD_PAGE1), (uint32_t*)&g_local_password, sizeof(g_local_password));  //Ð´Èëflash	
    if(g_FIFO_queue.head - g_FIFO_queue.tail != g_FIFO_queue.count){
		g_FIFO_queue.count = g_FIFO_queue.head - g_FIFO_queue.tail;
		FLASH_Write(FLASH_QUEUE_START_ADDR, (uint32_t*)&g_FIFO_queue, sizeof(g_FIFO_queue));
	}	
	
	ekey_flash_read(FLASH_LOG_START_ADDR, sizeof(g_open_log), (uint32_t*)&g_open_log, 1);
	ekey_flash_read(FLASH_QUEUE_START_ADDR, sizeof(g_FIFO_queue), (uint32_t*)&g_FIFO_queue, 2);
	ekey_flash_read(FLASH_PASSWORD_START_ADDR, sizeof(g_local_password), (uint32_t*)&g_local_password, 3);	
}


//--------------------------------------------------------------------------------------------
//  Section 发送开门日志部分: 发送日志函数封装API
//--------------------------------------------------------------------------------------------
void log_tx(void)
{
	static uint8 send_log_counter = 0;
	
	if(g_FIFO_queue.count == 0)
		return;
	if(g_resend_count > MAX_RESEND_TIME)
		return;
	
	send_log_counter++;
	
	if(send_log_counter >= 4){
		if(g_resend_count == MAX_RESEND_TIME)
			time_request();
		else
			send_open_log(g_endevice_info.server_addr);	
		
		send_log_counter=0;				
		g_resend_count ++;
	}		  	
}

//--------------------------------------------------------------------------------------------
//  Section 刷卡处理部分:发送开门日志
//--------------------------------------------------------------------------------------------
static void send_open_log(uint8_t *gwmac)
{	
    mes_to_updata_t mes_to_gateway;
	uint8_t tmp_msg[sizeof(mes_to_gateway)-2];
	
	mes_to_gateway.protocol_type[0] = 0xb0;
	mes_to_gateway.protocol_type[1] = 0xc0;
	mes_to_gateway.datalength[0] = 0x00; 
	mes_to_gateway.datalength[1] = sizeof(mes_to_gateway)-4;
	mes_to_gateway.nodemac[0] = 0x1a;
	mes_to_gateway.nodemac[1] = 0xaa;	 
	mes_to_gateway.nodemac[2] = DeviceType;
	mes_to_gateway.nodemac[3] = DevcMAC4;
	mes_to_gateway.nodemac[4] = DevcMAC5;
	mes_to_gateway.nodemac[5] = DevcMAC6;
	mes_to_gateway.cmd = 0x07;
	mes_to_gateway.payloadlength = 0x0c; //12bytes
	mes_to_gateway.device_type = 0x02; //��
	mes_to_gateway.protocol_ID[0] = 0x11;
	mes_to_gateway.protocol_ID[1] = 0x22;
	mes_to_gateway.final = 0xFF;
	
	memcpy(mes_to_gateway.gwmac, gwmac, 6);
	memcpy(mes_to_gateway.uid, g_open_log[g_FIFO_queue.tail].uid, 4);
	memcpy(mes_to_gateway.time, &g_open_log[g_FIFO_queue.tail].year, 6);
	mes_to_gateway.ifopen = g_open_log[g_FIFO_queue.tail].ifopen;
		
	memcpy(&tmp_msg, &mes_to_gateway, sizeof(mes_to_gateway) - 2);
	mes_to_gateway.checksum = bochiot_check_str(tmp_msg, sizeof(mes_to_gateway) - 2);

	bochiot_send_msg((uint8*)&mes_to_gateway, 36);
	
}

////--------------------------------------------------------------------------------------------
////  Section 删除卡号处理: 删除卡号
////--------------------------------------------------------------------------------------------
//void del_card_id(uint8_t *del_uid)
//{
//	for(int i =0; i<MAX_CARD_NUM; i++){
//		if(memcmp(del_uid,&g_local_password[i].uid,4)==0){ //判断卡号是否存在		
//				g_local_password[i].flag[0] = 0;		
//		}	
//	}
//}

////--------------------------------------------------------------------------------------------
////  Section删除卡号处理: 删除卡号响应
////--------------------------------------------------------------------------------------------
//void del_card_ask(uint8_t *gwmac,uint8_t *uid)
//{
//	uint8_t send_msg[29];
//	mes_to_updata_t mes_to_gateway;
//	
//	mes_to_gateway.protocol_type[0] = 0xb0;
//	mes_to_gateway.protocol_type[1] = 0xc0;
//	mes_to_gateway.datalength[0] = 0x00; //36bytes
//	mes_to_gateway.datalength[1] = 0x19;//sizeof(g_mes_to_gateway)-4;
//	mes_to_gateway.nodemac[0] = 0x1a;
//	mes_to_gateway.nodemac[1] = 0xaa;	 
//	mes_to_gateway.nodemac[2] = DeviceType;
//	mes_to_gateway.nodemac[3] = DevcMAC4;
//	mes_to_gateway.nodemac[4] = DevcMAC5;
//	mes_to_gateway.nodemac[5] = DevcMAC6;
//	mes_to_gateway.cmd = cmd_del_card;
//	mes_to_gateway.payloadlength = 0x07; //7bytes
//	mes_to_gateway.device_type = 0x02; //��
//	mes_to_gateway.protocol_ID[0] = 0x11;
//	mes_to_gateway.protocol_ID[1] = 0x22;

//	memcpy(mes_to_gateway.uid,uid,4);
//	memcpy(mes_to_gateway.gwmac,gwmac,6);
//	
//	memcpy(&send_msg,&mes_to_gateway,27);//除了checksum和0xff都复制过来;
//	send_msg[27] = bochiot_check_str(send_msg,sizeof(send_msg)-2);
//	send_msg[28] = 0xFF;
//	bochiot_send_msg((uint8*)&send_msg,29);
//}
