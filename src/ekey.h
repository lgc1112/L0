#ifndef  _EKEY_H_
#define  _EKEY_H_

#include "stm32l0xx_hal.h"
#include "stm32l0xx_nucleo.h"
#include "stm32l0xx_hal_rtc.h"
#include "bochiot_protocol.h"
#include "main.h"
#include "rtc.h"
#include "flash.h"


#define  MAX_LOG_NUM                     16
#define  MAX_RESEND_TIME                 5
#define  MAX_CARD_NUM					 5

// RTC define
#define RTC_TIMESTAMPPIN_DEFAULT              ((uint32_t)0x00000000U)
#define RTC_FORMAT_BIN   ((uint32_t)0x000000000U)
#define RTC_FORMAT_BCD   ((uint32_t)0x000000001U)

//刷卡模块返回宏定义
#define RSP_I2C_MSG_KEY_TYPE   0x81
#define RSP_I2C_MSG_SLEEP_TYPE 0x82
#define RSP_I2C_MSG_WAKE_TYPE  0x83
#define RSP_I2C_MSG_LED_TYPE   0x84
#define RSP_I2C_MSG_CARD_A     0x8A
#define RSP_I2C_MSG_CARD_B     0x8B
#define RSP_I2C_MSG_CARD_X     0x8C

#define REQ_I2C_MSG_SLEEP_CMD  0x02
#define REQ_I2C_MSG_WAKE_CMD   0x03
#define REQ_I2C_MSG_LED_CMD    0x04

#define SYS_STATUS_CHANGE_FROM_KEY     0x01
#define SYS_STATUS_CHANGE_FROM_CARD    0x02
#define SYS_STATUS_CHANGE_FROM_I2C     0x03
#define SYS_STATUS_CHANGE_FROM_TIMEOUT 0x04

typedef struct
{
	uint8_t protocol_type[2];// 0xc0b0
	uint8_t datalength[2]; //2bytes
	uint8_t gwmac[6];
	uint8_t nodemac[6];
	uint8_t nodetype[2];
	uint8_t protocol_ID[2];
	uint8_t cmd;
	uint8_t payloadlength;
	uint8_t device_type;
	uint8_t uid[4];
	uint8_t ifopen;
	uint8_t time[6];
	uint8_t checksum;
	uint8_t final;
}mes_to_updata_t;

typedef struct
{

	uint8_t head;
	uint8_t tail;
	uint8_t count;
	uint8_t pad;

}flash_queue_t;

typedef struct
{	
	
	uint8_t uid[4];
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t ifopen;
	uint8_t pad;
	uint8_t pad1[4];
}local_log_t;

typedef struct 
{

	uint8_t uid[4];
	uint8_t expire_time[6]; 
	uint8_t flag[6];
}flash_password_t;


//API
void swipe_card_handling(uint8_t* mes);
void refresh_password_status(void);
void add_card_handling(uint8 *add_card_uid, uint8 *expire_time, uint8_t *gwmac);
void del_open_log(uint8_t *gw_time2);
void ekey_flash_init(void);
void log_tx(void);

//内部处理函数
static void send_open_log(uint8_t *gwmac);
static void add_card_ask(uint8_t *gwmac, uint8_t *uid);	
static int get_write_card_idx(uint8_t* recv_uid);
static void get_opencard_id(uint8_t* mes, uint8_t *id);
static void add_open_log(uint8_t* uid,int flag);
static uint8_t judge_pwd_valid(uint8_t *id);
static void enqueue_open_log(void);
static void dequeue_open_log(void);
//static void del_card_id(uint8_t *del_uid);
//static void del_card_ask(uint8_t *gwmac,uint8_t *uid)








#endif
