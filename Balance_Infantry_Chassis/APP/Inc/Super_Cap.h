#ifndef __SUPER_CAP_H
#define __SUPER_CAP_H

#include "main.h"

//INCLUDE²¿·Ö
#include "PID.h"
#include "bsp_can.h"

typedef struct mb_reg_type
{
   uint16_t voltage;         
    int16_t chassis_power;
   uint16_t referee_power;       
   uint16_t reserve;       
}pm_od_t;

void Pm_Power_Set(FDCAN_HandleTypeDef *hfdcan,uint16_t power,uint16_t buffer_energy);

extern volatile pm_od_t pm_od;
extern int super_link[3];

#endif
