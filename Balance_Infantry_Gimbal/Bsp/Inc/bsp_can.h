#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

#include "fdcan.h"

void FDCAN_Init(void);

uint8_t Can_TxMessage(FDCAN_HandleTypeDef *hfdcan,uint32_t id,uint8_t len,uint8_t *data);

#endif

