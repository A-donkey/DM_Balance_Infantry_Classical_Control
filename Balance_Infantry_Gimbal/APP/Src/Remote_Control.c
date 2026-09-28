#include "Remote_Control.h"

//EXTERN部分
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;
//全局变量定义部分
RC_Ctrl_t Rc_Ctrl;
PC_Ctrl_t Pc_Ctrl;

/*******************************************************************************************************
遥控器初始化
********************************************************************************************************/
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

void Remote_Control_Init(void)
{
	RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/*******************************************************************************************************
遥控器数据解析
********************************************************************************************************/
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_Ctrl_t *rc_ctrl)
{
	if (sbus_buf == NULL || rc_ctrl == NULL)
	{
		return;
	}

	rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
	rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
	rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
											 (sbus_buf[4] << 10)) &0x07ff;
	rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
	rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
	rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
	rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
	rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
	rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
	rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left  Is Press ?
	rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
	rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
	rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

	rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
	
	//判断遥控器是否离线
	if(rc_ctrl->rc_link[0]>=1000) rc_ctrl->rc_link[0]=0;
	else rc_ctrl->rc_link[0]++;
}

void SBUS_TO_RC(void)
{
    DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)hdma_uart5_rx.Instance;

    // 清除错误标志
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_ORE))  __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF);
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_FE))   __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_FEF);
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_PE))   __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_PEF);

    // 空闲中断处理
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_IDLEFLAG(&huart5); // 清除 IDLE 标志

        if ((dma_stream->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_uart5_rx);
            while (dma_stream->CR & DMA_SxCR_EN); // 等待 DMA 真正停止

            this_time_rx_len = SBUS_RX_BUF_NUM - dma_stream->NDTR;
            dma_stream->NDTR = SBUS_RX_BUF_NUM;
            dma_stream->CR |= DMA_SxCR_CT;

            // 清除 DMA1 Stream1 标志位，确保能重新启动
            DMA1->LIFCR = (0x3DUL << 6);

            __HAL_DMA_ENABLE(&hdma_uart5_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &Rc_Ctrl);
            }
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_uart5_rx);
            while (dma_stream->CR & DMA_SxCR_EN); // 等待 DMA 真正停止

            this_time_rx_len = SBUS_RX_BUF_NUM - dma_stream->NDTR;
            dma_stream->NDTR = SBUS_RX_BUF_NUM;
            dma_stream->CR &= ~(DMA_SxCR_CT);

            // 清除 DMA1 Stream1 标志位，确保能重新启动
            DMA1->LIFCR = (0x3DUL << 6);

            __HAL_DMA_ENABLE(&hdma_uart5_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1], &Rc_Ctrl);
            }
        }
        
        // 保证 DMAR 接收请求始终开启
        SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);
    }
}
