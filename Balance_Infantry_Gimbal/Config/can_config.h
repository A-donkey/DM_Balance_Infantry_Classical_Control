#ifndef __CAN_CONFIG_H
#define __CAN_CONFIG_H

//零点设定：前方电机大腿向后平行，后方电机短杆向前平行

/********************CHASSIS_CAN1*************************/
#define LEFT_FRONT_MOTOR_CTRL_ID        	0x03
#define LEFT_BACK_MOTOR_CTRL_ID        		0x04
#define LEFT_FRONT_MOTOR_FEEDBACK_ID    	0x10
#define LEFT_BACK_MOTOR_FEEDBACK_ID    		0x11
#define LEFT_WHEEL_MOTOR_CTRL_ID          0x200
#define LEFT_WHEEL_MOTOR_FEEDBACK_ID      0x202

/*********************CHASSIS_CAN2************************/
#define RIGHT_FRONT_MOTOR_CTRL_ID       	0x05
#define RIGHT_BACK_MOTOR_CTRL_ID       		0x06
#define RIGHT_FRONT_MOTOR_FEEDBACK_ID   	0x12
#define RIGHT_BACK_MOTOR_FEEDBACK_ID   		0x13
#define RIGHT_WHEEL_MOTOR_CTRL_ID         0x200
#define RIGHT_WHEEL_MOTOR_FEEDBACK_ID     0x201
#define CHASSIS_TO_CAP_CAN_ID             0x50
#define CAP_TO_CHASSIS_CAN_ID             0x51

/*********************GIMBAL_CAN2************************/
#define PITCH_MOTOR_CTRL_ID               0x0C
#define PITCH_MOTOR_FEEDBACK_ID           0x0D
#define FRICTION_WHEELS_CTRL_ID           0x1FF
#define LEFT_FRICTION_WHEEL_FEEDBACK_ID   0x207
#define RIGHT_FRICTION_WHEEL_FEEDBACK_ID  0x208

/**************CHASSIS_CAN3 && GIMBAL_CAN3***************/
#define CHASSIS_TO_GIMBAL_CAN_ID_1        0x105
#define GIMBAL_TO_CHASSIS_CAN_ID_1        0x100
#define GIMBAL_TO_CHASSIS_CAN_ID_2        0x120
#define DT7_TO_CHASSIS_CAN_ID             0x80
#define YAW_MOTOR_CTRL_ID                 0x0A
#define YAW_MOTOR_FEEDBACK_ID             0x0B
#define TOGGLE_MOTOR_FEEDBACK_ID         	0x206
#define TOGGLE_MOTOR_CTRL_ID							0x1FF
#endif
