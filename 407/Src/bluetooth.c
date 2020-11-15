#include "main.h"
#include "stm32f4xx_hal.h"
#include "config.h"
#include "bluetooth.h"
#include "stdio.h"

extern UART_HandleTypeDef UART_PORT1;
extern UART_HandleTypeDef UART_PORT2;
extern UART_HandleTypeDef UART_PORT3;
uint8_t bt_cnt = 0;
uint8_t usart_cnt = 0;
uint8_t btRxbuf[3] = {0};
uint8_t usartTxbuf[8] = {0};
uint8_t message[1] = {0};

void usart_task(speed3axistype *speed)
{
	usartTxbuf[0] = 0x31;
	usartTxbuf[1] = 0x30;
	usartTxbuf[2] = speed->x0;
	usartTxbuf[3] = speed->x;
	usartTxbuf[4] = speed->y0;
	usartTxbuf[5] = speed->y;
	usartTxbuf[6] = speed->r0;
	usartTxbuf[7] = speed->r;
	HAL_UART_Transmit_DMA(&UART_PORT2, (uint8_t *)usartTxbuf, 8);
}



//void BT_task()
//{
//	/*state mechime : 
//	0:free ,set dma
//	1:wait to recieved start frame 
//	2:start frame recieved,set dma for body
//	3:waiting for body(dma)
//	4:data recieved complete*/	
//	switch(bt_cnt)
//	{
//		case 0:
//			btRxbuf[0] = 0;
//			HAL_UART_Receive_DMA(&UART_PORT1,btRxbuf,1);
//			bt_cnt = 1;
//		  break;
//		case 1:
//			break;
//		case 2:
//			HAL_UART_Receive_DMA(&UART_PORT1,&btRxbuf[1],2);
//			bt_cnt = 3;
//		  break;
//		case 3:
//		  break;
//		case 4:
//			if(0x30 == btRxbuf[1]){
//				message[1] = 'A';
//				HAL_UART_Transmit_DMA(&UART_PORT1, (uint8_t *)message, 1);
//				printf("test message\n");		
//			bt_cnt = 0;
//			break;
//			}
//			bt_cnt = 0;
//		default:
//			bt_cnt = 0;		
//	}
//	
////	#if OFFLINE_DECT == 1
////	if(speed->time+TIMEOUT_TH <=  HAL_GetTick())
////	{
////		speed->y = 0;
////		speed->x = 0;
////		speed->r = 0;
////		speed->cal_speed = 1;
////		speed->time = HAL_GetTick();
////	}
////	#endif
//}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	
}
