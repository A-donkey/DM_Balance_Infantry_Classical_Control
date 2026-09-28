#include "Buzzer_Run.h"

//全局变量定义部分
bool Ready_Buzzer_Flag;
bool pc_first_connected = false;  // 首次连接标志
uint16_t pc_beep_timer     = 100; // 蜂鸣器关闭倒计时
/*******************************************************************************************************
机器人初始化完成提示音
********************************************************************************************************/
void Ready_Buzzer(void)
{
	static int Ready_Buzzer_Time = 0;
	
	if(Ready_Buzzer_Flag == 0)
	{
		Buzzer_On(1,5000);
		
		Ready_Buzzer_Time++;
		
		if(Ready_Buzzer_Time>=1000)
		{
			Ready_Buzzer_Flag = 1;
			Ready_Buzzer_Time = 0;
			
			Buzzer_Off();
		}
	}
}

/*******************************************************************************************************
蜂鸣器错误警报
********************************************************************************************************/
void Error_Buzzer(Link_Sit_t *link)
{
	static int buzzer_time = 0; //鸣叫计时
	
	//[bz:0为长鸣次数 1为短鸣次数]
	static int buzzer_num[2] = {0,0};  //鸣叫次数
	static int buzzer_comp[2] = {0,0}; //完成的鸣叫次数
	
	static bool cycle_flag = 0; //周期鸣叫完成标志位
	
	if(!cycle_flag) //周期开启前设置鸣叫次数
	{
		if(link->err_num != 0)
		{
			buzzer_num[0] = 0;
			buzzer_num[1] = 0;
		}
		else
		{
			buzzer_num[0] = 0;
			buzzer_num[1] = 0;
		}
		
		buzzer_time = 0;
	}
	
	if(buzzer_comp[0]!=buzzer_num[0] || buzzer_comp[1]!=buzzer_num[1]) //开启鸣叫周期
	{
		cycle_flag = 1;
		
		if(buzzer_comp[0] != buzzer_num[0]) //先长鸣
		{
			switch(buzzer_time%100)
			{
				case 0:
				{
					buzzer_time++;
					Buzzer_On(1,5000);
					break;
				}
				case 40:
				{
					buzzer_time++;
					Buzzer_Off();
					break;
				}
				default:
				{
					buzzer_time++;
					if(buzzer_time == 99) 
					{
						buzzer_time = 0;
						buzzer_comp[0]++;
					}
					break;
				}
			}
		}
		else if(buzzer_comp[1] != buzzer_num[1]) //后短鸣
		{
			switch(buzzer_time%40)
			{
				case 0:
				{
					buzzer_time++;
					Buzzer_On(1,5000);
					break;
				}
				case 20:
				{
					buzzer_time++;
					Buzzer_Off();
					break;
				}
				default:
				{
					buzzer_time++;
					if(buzzer_time == 39) 
					{
						buzzer_time = 0;
						buzzer_comp[1]++;
					}
					break;
				}
			}
		}
	}
	else if(buzzer_comp[0]==buzzer_num[0] && buzzer_comp[1]==buzzer_num[1]) //鸣叫周期结束,等待片刻后开启下一周期
	{
		buzzer_time++;
		
		if(buzzer_time>=240)
		{
			cycle_flag = 0;
			buzzer_time = 0;
			
			buzzer_comp[0] = 0;
			buzzer_comp[1] = 0;
		}
	}
}

/*******************************************************************************************************
成功建立通信提示
********************************************************************************************************/
void Recieve_PC_Success_Buzzer(void){
	if(pc_first_connected && pc_beep_timer>0) {
    Buzzer_On(3, 5000);           // 开
    pc_beep_timer--;
    if(pc_beep_timer == 0)
        Buzzer_Off();              // 300周期后才关
		}
}

