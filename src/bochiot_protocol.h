#ifndef __BOCHIOT_PROTOCOL_H
#define __BOCHIOT_PROTOCOL_H

//#include "stm32l1xx_nucleo.h"
#include "main.h"
#include "ekey.h"
//bochiot_protocol.h






#define PROTOCOL_TYPE  0xc0b0
#define MAC_LEN			4
#define MAC_GW_LEN		6
#define SIZE_proto_type		2
#define SIZE_data_len		2
#define SIZE_node_type		2
#define SIZE_protocol		2
#define SIZE_crc			2
#define SIZE_mac_number	1

#define SENSOR_XYZ_COUNT	3
#define COMMON_VALUE		64500
#define D_Value_Z			60
#define D_Value_Y			60
#define D_Value_X			60

#define Lock_Node                    0x02
#define Electrical_Switch        0x03

#define RcvPack		01        //解包时的读取节点地址调用
#define	BackPack    02		  //回包时的读取节点地址调用

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;

typedef void (*Func_callBack)(uint8 *msg, uint8 msg_len);

typedef enum{
	node_gateway=0,
	node_parking_spot=1,
	node_parking_display=2,
}node_type;

typedef enum{
	cmd_register=0x01,
	cmd_heart_beat=0x02,
	cmd_report_msg=0x03,
	cmd_ctrl_msg=0x04,
	cmd_get_voltage=0x05,
	cmd_add_card=0x06,
	cmd_del_log=0x07,
	cmd_synchronize_tim=0x08,
	cmd_add_secret=0x09,
	cmd_del_card=0x0a,
}cmd_type;


typedef struct _st_payload_data
{
	uint8  cmd_type;
	uint8  payload_data_len;
	uint8  pData[1];  // can't be zero here
}__attribute__ ((packed))MAC_PAYLOAD_DATA;

/* 收包时的包头结构 */
typedef struct _st_frame_head_RX       
{
	uint16 protocol_type;
	uint8 frame_data_len[2];
	uint8 Mac_Number;
}__attribute__ ((packed))MAC_FRAME_HEAD_RX;

typedef struct _st_frame_head
{
	uint16 protocol_type;
	uint8 frame_data_len[2];
	uint8  dst_adr[MAC_GW_LEN];
	uint8  src_adr[MAC_GW_LEN];
	uint16  node_type;
	uint16  protocol_ID;
}__attribute__ ((packed))MAC_FRAME_HEAD;


typedef struct _st_frame_data
{
	MAC_FRAME_HEAD    frame_head;
	MAC_PAYLOAD_DATA *payload_data;
	uint32  mac_crc;
}__attribute__ ((packed))MAC_FRAME_DATA;






//Mac地址结构体，用于储存节点Mac和网关Mac地址

typedef struct _st_mac_info
{
	uint8 jiedianMac[6];
	uint8 wangguanMac[6];

}__attribute__ ((packed))MAC_INFO;


extern void bochiot_protocol_init(Func_callBack send_msg);
extern int bochiot_anaylyze_frame(uint8 * frame_buf,uint8 buf_len);
extern int bochiot_report_status(int flag);
extern void bochiot_request_register(void);
extern int bochiot_check_register(void);
extern int bochiot_heart_beat(void);
extern int bochiot_set_localAddr(uint8 * set_Addr);
extern int bochiot_GetMacConfigStatus(void);
extern int bochiot_SetMacConfig(char* mac);
uint8 bochiot_check_str(uint8 *retPacket,uint8 len);


#endif

