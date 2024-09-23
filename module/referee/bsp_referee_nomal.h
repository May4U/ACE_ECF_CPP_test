#ifndef __BSP_REFEREE_NOMAL_H
#define __BSP_REFEREE_NOMAL_H

#ifdef __cplusplus   //这里是两个下滑线
extern "C"{
#endif

#include "CRC.h"
#include "stdint.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "bsp_referee.h"


//裁判系统初始化
void referee_uart_init_nomal(UART_HandleTypeDef *huart,uint8_t *rx_buffer1, uint8_t *rx_buffer2, uint8_t buff_num);
void ECF_referee_uart_init_nomal(void);
void REFEREE_UART_IRQHandler_nomal(UART_HandleTypeDef *huart);
REFEREE_t *Get_referee_Address_nomal(void);

#ifdef __cplusplus
}
#endif

#endif
