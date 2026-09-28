#ifndef __BUZZER_RUN_H
#define __BUZZER_RUN_H

#include "main.h"

//INCLUDE部分
#include "stdbool.h"
#include "bsp_buzzer.h"
#include "Check_Task.h"

void Ready_Buzzer(void);

//EXTERN部分
extern bool Ready_Buzzer_Flag;
extern bool     pc_first_connected;  // 首次连接标志
extern uint16_t pc_beep_timer;       // 蜂鸣器关闭倒计时
#endif




