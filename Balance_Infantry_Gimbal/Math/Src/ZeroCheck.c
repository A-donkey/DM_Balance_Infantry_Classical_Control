#include "ZeroCheck.h"

/**
 * @brief 过零检测
 * @param[in] Zero 过零检测结构体
 * @param[in] value 当前检测值
 */
float ZeroCheck(ZeroCheck_Typedef *Zero, float value, float CountCycle,float threshold ,float speed)
{
	Zero->CountCycle = CountCycle;
	Zero->ActualValue = value;

	Zero->PreError = Zero->ActualValue - Zero->LastValue;
	Zero->LastValue = Zero->ActualValue;
	
	if (Zero->PreError > threshold * Zero->CountCycle)
	{
//		if(speed < 0)
//		{
//			Zero->PreError = Zero->PreError - Zero->CountCycle;
//			Zero->Circle++;
//		}
//		else if(speed > 0)
//		{
//			Zero->PreError = Zero->PreError + Zero->CountCycle;
//			Zero->Circle--;
//		}
		Zero->PreError = Zero->PreError - Zero->CountCycle;
		Zero->Circle++;
	}
	if (Zero->PreError < -threshold * Zero->CountCycle)
	{
//		if(speed > 0)
//		{
//			Zero->PreError = Zero->PreError + Zero->CountCycle;
//			Zero->Circle--;
//		}
//		else if(speed < 0)
//		{
//			Zero->PreError = Zero->PreError - Zero->CountCycle;
//			Zero->Circle++;
//		}
		Zero->PreError = Zero->PreError + Zero->CountCycle;
		Zero->Circle--;
	}
			
	return Zero->ActualValue - Zero->Circle * Zero->CountCycle;
}
