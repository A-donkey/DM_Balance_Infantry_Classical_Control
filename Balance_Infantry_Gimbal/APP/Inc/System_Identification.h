#ifndef __SYSTEM_IDENTIFICATION_H__
#define __SYSTEM_IDENTIFICATION_H__

#include "Gimbal_Task.h"
#include "PID.h"                    
#include "RLS_Identification.h"     
#include "Can_Feedback.h"           
#include "user_lib.h"               
#include <math.h>                   

void Gimbal_Pitch_SysID_Run(Gimbal_Status_t *gs);
void Gimbal_Yaw_SysID_Run(Gimbal_Status_t *gs);

#endif
