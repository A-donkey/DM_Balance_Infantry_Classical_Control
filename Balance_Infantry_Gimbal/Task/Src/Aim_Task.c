#include "Aim_Task.h"

//INCLUDE部分
#include "Ins_Task.h"
#include "Check_Task.h"
#include "Gimbal_Task.h"
#include "Can_Feedback.h"
//全局变量定义部分
Aim_Rx aim_rx;
Aim_Tx aim_tx;

static uint8_t usb_tx_buf[APP_TX_DATA_SIZE];

bool aim_ready_flag; //当前线程初始化完成标志 

/*******************************************************************************************************
Aim任务初始化
********************************************************************************************************/
void Aim_Init(void)
{
	aim_ready_flag = 1;
	
	//等待
	while(!all_ready_flag) {osDelay(1);}
}

/*******************************************************************************************************
Aim任务
********************************************************************************************************/
void Aim_Task(void)
{
	Send_Packet();
}

/*******************************************************************************************************
向上位机发送数据
********************************************************************************************************/
void Send_Packet(void)
{
	aim_tx.frame_header1 = TX_HEADER1;
	aim_tx.frame_header2 = TX_HEADER2;

  aim_tx.pitch_now 						= Gimbal_Status.pitch;
  aim_tx.yaw_now 							= Gimbal_Status.yaw;
  aim_tx.pitch_omega 					= -INS.Gyro[0] * PI_Ang;
  aim_tx.yaw_omega 						= INS.Gyro[2] * PI_Ang;
  aim_tx.pitch_tff 						= Gimbal_Motor.DM_4310[1].tor;
  aim_tx.yaw_tff 							= Gimbal_Motor.DM_4310[0].tor;
  aim_tx.actual_bullet_speed 	= 23.0f; //暂时写为硬编码,后期更改
  aim_tx.shoot_avaiable 			= Heat_Control.Perm_Bullets_Num;
  aim_tx.aim_request 					= Aim_Permission;
  // mode_want在其他地方处理
  if (aim_tx.mode_want == NOT_USE_AIM)
  {
     aim_tx.mode_want = STD_AUTO_AIM;//算法要求，默认发1
  }
  aim_tx.number_want = 0;
  aim_tx.enemy_color = Down_Cboard_Info.enem_color;
  aim_tx.crc16 = CRC16_Calculate((uint8_t *)(&aim_tx),PC_SENDBUF_SIZE - 3);// 不包含frame_tail
  aim_tx.frame_tail = 0x0D;
	
	//发送
  memcpy(usb_tx_buf,&aim_tx,PC_SENDBUF_SIZE);
  CDC_Transmit_HS(usb_tx_buf,PC_SENDBUF_SIZE);
}

/*******************************************************************************************************
接收上位机数据
********************************************************************************************************/
void Recieve_Host(uint8_t* buff) 
{
	if(buff[0] == RX_HEADER1 && buff[1] == RX_HEADER2 
	&& buff[PC_RECVBUF_SIZE - 1] == FRAME_TAIL) //与接收到的帧头一致
	{
		//如果CRC校验通过则保留数据,否则只保留上一帧数据
		if(CRC16_Verify((uint8_t *)(buff), PC_RECVBUF_SIZE-1)) {
			if(!pc_first_connected && pc_beep_timer > 0){
				pc_first_connected = true;
			}
			memcpy(&aim_rx,buff,PC_RECVBUF_SIZE);
		}
	}	
}
