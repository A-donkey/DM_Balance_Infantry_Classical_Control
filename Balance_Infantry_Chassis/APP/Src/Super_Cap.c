#include "Super_Cap.h"
#include "Power_Limit.h"

//全局变量定义部分
volatile pm_od_t pm_od;

int super_link[3] = { 0 };

/*******************************************************************************************************
设定功率值
********************************************************************************************************/
void Pm_Power_Set(FDCAN_HandleTypeDef *hfdcan,uint16_t power,uint16_t buffer_energy)
{
	uint8_t data[8];
	
	data[0] = (uint8_t)(buffer_energy &  0xFF);
	data[1] = (uint8_t)(buffer_energy >>    8);
	data[2] = (uint8_t)(power &  0xFF);
	data[3] = (uint8_t)(power >>    8);
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;

	Can_TxMessage(hfdcan,0x50,0x08,data);
}
