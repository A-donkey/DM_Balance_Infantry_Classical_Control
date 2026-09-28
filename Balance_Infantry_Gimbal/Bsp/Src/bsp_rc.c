#include "bsp_rc.h"

extern UART_HandleTypeDef huart5 ;
extern DMA_HandleTypeDef hdma_uart5_rx;

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)hdma_uart5_rx.Instance;
	
	// 1. 禁用空闲中断并清除历史错误标志
	__HAL_UART_DISABLE_IT(&huart5, UART_IT_IDLE);
	__HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_IDLEF);
	(void)huart5.Instance->RDR; // 读出可能残留的字节清 RXNE

	// 2. 停用 DMA
	__HAL_DMA_DISABLE(&hdma_uart5_rx);
	while(dma_stream->CR & DMA_SxCR_EN);

	// 3. 清除 DMA1 Stream1 所有的历史中断标志 (防止使能失败)
	DMA1->LIFCR = (0x3DUL << 6);

	// 4. 配置外设及双缓冲内存地址
	dma_stream->PAR = (uint32_t) & (UART5->RDR);
	dma_stream->M0AR = (uint32_t)(rx1_buf);
	dma_stream->M1AR = (uint32_t)(rx2_buf);
	dma_stream->NDTR = dma_buf_num;

	// 5. 配置双缓冲模式并将起始目标设为 Memory 0
	SET_BIT(dma_stream->CR, DMA_SxCR_DBM);
	dma_stream->CR &= ~DMA_SxCR_CT;

	// 6. 使能 DMA
	__HAL_DMA_ENABLE(&hdma_uart5_rx);

	// 7. 开启串口 DMA 请求与空闲中断
	SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
}
