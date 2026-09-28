#ifndef __SUPER_CAP_TASK_H
#define __SUPER_CAP_TASK_H

#include "main.h"

//INCLUDE部分
#include "PID.h"
#include "stdbool.h"
#include "Referee.h"
#include "Super_Cap.h"
#include "Check_Task.h"
#include "Power_Limit.h"

void Super_Cap_Init(void);
void Super_Cap_Task(void);
void Super_Cap_Control(Controlled_State_t *cs);

//EXTERN部分
extern bool super_ready_flag;

#endif
