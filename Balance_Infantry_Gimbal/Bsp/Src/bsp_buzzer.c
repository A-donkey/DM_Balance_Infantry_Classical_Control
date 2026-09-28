#include "bsp_buzzer.h"

/*******************************************************************************************************
·äÃùÆ÷¿ª¹Øº¯Êý
********************************************************************************************************/
void Buzzer_On(uint16_t psc, uint16_t pwm)
{
	__HAL_TIM_PRESCALER(&htim12, psc);
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, pwm);
}

void Buzzer_Off(void)
{
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, 0);
}

