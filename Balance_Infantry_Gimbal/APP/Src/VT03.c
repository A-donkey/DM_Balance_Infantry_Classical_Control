#include "VT03.h"

//INCLUDE部分
#include "CRCs.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
//EXTERN部分
extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_rx;
//全局变量定义部分

int vt_link[3] = {0,0,0};//遥控器连接检测数组

remote_data_t VT03;

/*******************************************************************************************************
VT03初始化
********************************************************************************************************/
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t vt03_rx_buf[2][VT03_RX_BUF_NUM];

void VT03_Init(void)
{
    DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)hdma_uart7_rx.Instance;
    
    // 清除串口历史错误及标志
    __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    __HAL_UART_CLEAR_IDLEFLAG(&huart7);
    (void)huart7.Instance->RDR;
    SET_BIT(huart7.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
    __HAL_DMA_DISABLE(&hdma_uart7_rx);
    while (dma_stream->CR & DMA_SxCR_EN);
    // 清除 DMA2 Stream1 标志位 (bit 6~11)
    DMA2->LIFCR = (0x3DUL << 6);
    dma_stream->PAR = (uint32_t)&(UART7->RDR);
    dma_stream->M0AR = (uint32_t)(vt03_rx_buf[0]);
    dma_stream->M1AR = (uint32_t)(vt03_rx_buf[1]);
    dma_stream->NDTR = VT03_RX_BUF_NUM;
    SET_BIT(dma_stream->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(&hdma_uart7_rx);
}

/*******************************************************************************************************
VT03数据解析
********************************************************************************************************/

void RX_TO_VT03(void)
{
    DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)hdma_uart7_rx.Instance;
    // 清除可能导致串口接收卡死锁死的 ORE / FE 标志
    if (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_ORE)) __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_OREF);
    if (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_FE))  __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_FEF);
    // 严禁使用 if(RXNE) 导致跳过 IDLE！直接判断 IDLE 空闲标志
    if (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_IDLE))
    {
        static uint16_t this_time_rx_len = 0;
        __HAL_UART_CLEAR_IDLEFLAG(&huart7); // 必须在进入时清除 IDLE 标志
        if ((dma_stream->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_uart7_rx);
            while (dma_stream->CR & DMA_SxCR_EN); // 等待 DMA 真正停止后再改写 NDTR
            this_time_rx_len = VT03_RX_BUF_NUM - dma_stream->NDTR;
            dma_stream->NDTR = VT03_RX_BUF_NUM;
            dma_stream->CR |= DMA_SxCR_CT;
            DMA2->LIFCR = (0x3DUL << 6); // 清除 DMA2 Stream1 标志
            __HAL_DMA_ENABLE(&hdma_uart7_rx);
            if (this_time_rx_len == VT03_FRAME_LENGTH)
            {
                if (CRC16_Verify((uint8_t *)(vt03_rx_buf[0]), sizeof(remote_data_t)))
                {
                    memcpy(&VT03, vt03_rx_buf[0], sizeof(remote_data_t));
                }
                if (vt_link[0] >= 1000) vt_link[0] = 0;
                else vt_link[0]++;
            }
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_uart7_rx);
            while (dma_stream->CR & DMA_SxCR_EN); // 等待 DMA 真正停止
            this_time_rx_len = VT03_RX_BUF_NUM - dma_stream->NDTR;
            dma_stream->NDTR = VT03_RX_BUF_NUM;
            dma_stream->CR &= ~(DMA_SxCR_CT);
            DMA2->LIFCR = (0x3DUL << 6); // 清除 DMA2 Stream1 标志
            __HAL_DMA_ENABLE(&hdma_uart7_rx);
            if (this_time_rx_len == VT03_FRAME_LENGTH)
            {
                if (CRC16_Verify((uint8_t *)(vt03_rx_buf[1]), sizeof(remote_data_t)))
                {
                    memcpy(&VT03, vt03_rx_buf[1], sizeof(remote_data_t));
                }
                if (vt_link[0] >= 1000) vt_link[0] = 0;
                else vt_link[0]++;
            }
        }
    }
}

