#ifndef __BOARD_CAN_TASK_H
#define __BOARD_CAN_TASK_H

#include "main.h"

//INCLUDE部分
#include "Check_Task.h"
#include "Can_Feedback.h"

void Send_Rc_DT7_1(void);
void Send_Rc_DT7_2(void);
void Send_Message_1(void);
void Send_Message_2(void);
void Board_Can_Init(void);
void Board_Can_Task(void);

//EXTERN部分
extern bool board_ready_flag;

#endif
