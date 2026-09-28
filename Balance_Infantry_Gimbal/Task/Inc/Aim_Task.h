#ifndef __AIM_TASK_H
#define __AIM_TASK_H

#include "main.h"

//INCLUDE部分
#include "CRCs.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "bsp_buzzer.h"
#include "Buzzer_Run.h"

#define RX_HEADER1 0xAA
#define RX_HEADER2 0x55
#define TX_HEADER1 0x55
#define TX_HEADER2 0xAA
#define FRAME_TAIL 0x0D

#define PC_SENDBUF_SIZE sizeof(Aim_Tx)
#define PC_RECVBUF_SIZE sizeof(Aim_Rx)

typedef enum AUTOAIM_MODE
{
	NOT_USE_AIM = 0,
	STD_AUTO_AIM,
	SMALL_BUFF,
	BIG_BUFF,
} AUTOAIM_MODE;

//发送数据定义
typedef struct Aim_Tx
{
	uint8_t frame_header1; // 0x55
	uint8_t frame_header2; // 0xAA
	float pitch_now;
	float yaw_now;
	float pitch_omega;				 // pitch角速度
	float yaw_omega;					 // yaw角速度
	float pitch_tff;					 // pitch力矩
	float yaw_tff;						 // yaw力矩
	float actual_bullet_speed; // 弹速
	uint8_t shoot_avaiable;		 // 可发弹量
	uint8_t aim_request;			 // 右键
	uint8_t mode_want;				 // 模式 辅瞄模式：0 大符模式：1 小符模式：2
	uint8_t number_want;			 // 不用，但是不能删，否则解码出问题
	uint8_t enemy_color;			 // 自己：蓝 1 红 0
	uint16_t crc16;
	uint8_t frame_tail; 			 // 0x0D
} __attribute__((packed))Aim_Tx;

//接收数据定义
typedef struct Aim_Rx
{
	uint8_t frame_header1; 			// 0xAA
	uint8_t frame_header2; 			// 0x55
	uint8_t mode_select;   			//控制模式选择 0x11为位置控制模式，0x22 为前馈力矩+位置+速度模式
	uint8_t detect_number; 			// 是否识别到;
	uint8_t shoot_flag;		 			// 上位机决定打弹与否
	float pitch_setpoint;				// 目标pitch
	float yaw_setpoint;			  	// 目标yaw
	float pitch_omega_setpoint;	// 目标pitch角速度
	float yaw_omega_setpoint;		// 目标yaw角速度
	float pitch_acc_setpoint;		// 目标pitch加速度
	float yaw_acc_setpoint;			// 目标yaw加速度
	uint16_t crc16;
	uint8_t frame_tail; 				// 0x0D
} __attribute__((packed))Aim_Rx;

void Aim_Init(void);
void Aim_Task(void);
void Send_Packet(void);
void Recieve_Host(uint8_t* buff);

//EXTERN部分
extern bool aim_ready_flag;

extern Aim_Rx aim_rx;
extern Aim_Tx aim_tx;

#endif

