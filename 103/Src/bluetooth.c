#include "main.h"
#include "stm32f1xx_hal.h"
#include "config.h"
#include "motor.h"
#include "bluetooth.h"
#include "string.h"
#include <stdio.h>

extern UART_HandleTypeDef UART_PORT;
//uint8_t bt_cnt = 0;
uint8_t usart_cnt = 0;
//uint8_t btRxbuf[3]={0};
uint8_t usartRxbuf[8]={0};
uint8_t message[1] = {0};

void usart_task(mt_ctrltype *ctrl,pidtype *mt,speed3axistype *speed)
{
	/*state mechime : 
	0:free ,set dma
	1:wait to recieved start frame 
	2:start frame recieved,set dma for body
	3:waiting for body(dma)
	4:data recieved complete*/	
	switch(usart_cnt)
	{
		case 0:
			usartRxbuf[0] = 0;
			HAL_UART_Receive_DMA(&UART_PORT,usartRxbuf,1);
			usart_cnt = 1;
		break;
		case 1:
			break;
		case 2:
			HAL_UART_Receive_DMA(&UART_PORT,&usartRxbuf[1],7);
			usart_cnt = 3;
		break;
			case 3:
		break;
		case 4:
			if(0x30 == usartRxbuf[1])
			{
        if(usartRxbuf[2]>=128){
					speed->x = 10*usartRxbuf[3] + (usartRxbuf[2]-128);
				}
				else if(usartRxbuf[2]<128){
					speed->x = -10*usartRxbuf[3] + (usartRxbuf[2]-128);
				}
				if(usartRxbuf[4]>=128){
					speed->y = 10*usartRxbuf[5] + (usartRxbuf[4]-128);
				}
				else if(usartRxbuf[4]<128){
					speed->y = -10*usartRxbuf[5] + (usartRxbuf[4]-128);
				}
				if(usartRxbuf[6]>=128){
					speed->r = 10*usartRxbuf[7] + (usartRxbuf[6]-128);
				}
				else if(usartRxbuf[6]<128){
					speed->r = -10*usartRxbuf[7] + (usartRxbuf[6]-128);
				}
//				speed->x = (usartRxbuf[2]-128)/abs(usartRxbuf[2]-128)*10*usartRxbuf[3] + (usartRxbuf[2]-128);
//        speed->y = (usartRxbuf[4]-128)/abs(usartRxbuf[4]-128)*10*usartRxbuf[5] + (usartRxbuf[4]-128);
//				speed->r = (usartRxbuf[6]-128)/abs(usartRxbuf[6]-128)*10*usartRxbuf[7] + (usartRxbuf[6]-128);				
				speed->cal_speed = 1;
				speed->time = HAL_GetTick();
			usart_cnt = 0;
			break;
			}
			usart_cnt = 0;
		default:
			usart_cnt = 0;		
	}
	
	#if OFFLINE_DECT == 1
	if(speed->time+TIMEOUT_TH <=  HAL_GetTick())
	{
		speed->y = 0;
		speed->x = 0;
		speed->r = 0;
		speed->cal_speed = 1;
		speed->time = HAL_GetTick();
	}
	#endif
}

//void BT_task(mt_ctrltype *ctrl,pidtype *mt,speed3axistype *speed)
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
//			HAL_UART_Receive_DMA(&UART_PORT,btRxbuf,1);
//			bt_cnt = 1;
//		break;
//		case 1:
//			break;
//		case 2:
//			HAL_UART_Receive_DMA(&UART_PORT,&btRxbuf[1],2);
//			bt_cnt = 3;
//		break;
//			case 3:
//		break;
//		case 4:
//			if(0x30 == btRxbuf[1])
//			{
//				if(btRxbuf[2] == 'f'){
//					message[0] = 'f';
//						speed->y = 500;//y
//						speed->x = 0;
//						speed->r = 0;
//						speed->cal_speed = 1;
//						speed->time = HAL_GetTick();
//				}
//				else if(btRxbuf[2] == 'b'){
//					  message[0] = 'b';
//					  speed->y = -500;//y
//						speed->x = 0;
//						speed->r = 0;
//						speed->cal_speed = 1;
//						speed->time = HAL_GetTick();
//				}
//				else if(btRxbuf[2] == 'l'){
//					  message[0] = 'l';
//					  speed->y = 0;//y
//						speed->x = -500;
//						speed->r = 0;
//						speed->cal_speed = 1;
//						speed->time = HAL_GetTick();
//				}
//				else if(btRxbuf[2] == 'r'){
//					  message[0] = 'r';
//					  speed->y = 0;//y
//						speed->x = 500;
//						speed->r = 0;
//						speed->cal_speed = 1;
//						speed->time = HAL_GetTick();
//				}
//				else if(btRxbuf[2] == 'L'){
//					  message[0] = 'L';
//					  speed->y = 0;//y
//						speed->x = 0;
//						speed->r = -200;
//						speed->cal_speed = 1;
//						speed->time = HAL_GetTick();
//				}
//				else if(btRxbuf[2] == 'R'){
//					  message[0] = 'R';
//					  speed->y = 0;//y
//						speed->x = 0;
//						speed->r = 200;
//						speed->cal_speed = 1;
//						speed->time = HAL_GetTick();
//				}
//				else if(btRxbuf[2] == 's'){
//					  message[0] = 's';
//					  speed->y = 0;//y
//						speed->x = 0;
//						speed->r = 0;
//						speed->cal_speed = 1;
//						speed->time = HAL_GetTick();
//				}
////				switch (btRxbuf[2])
////			  {
////					case 0x66:
////					{ 
////						
////            break;						
////					}
////					case 0x62:
////						
////					  break;
////					case 0x6c:
////						message[0] = 'l';
////					  speed->y = 0;//y
////						speed->x = -100;
////						speed->r = 0;
////						speed->cal_speed = 1;
////						speed->time = HAL_GetTick();
////					  break;
////					case 0x72:
////						message[0] = 'r';
////					  speed->y = 0;//y
////						speed->x = 100;
////						speed->r = 0;
////						speed->cal_speed = 1;
////						speed->time = HAL_GetTick();
////					  break;
////					case 0x4c:
////						message[0] = 'L';
////					  speed->y = 0;//y
////						speed->x = 0;
////						speed->r = 100;
////						speed->cal_speed = 1;
////						speed->time = HAL_GetTick();
////					  break;
////					case 0x52:
////						message[0] = 'R';
////					  speed->y = 0;//y
////						speed->x = 0;
////						speed->r = -100;
////						speed->cal_speed = 1;
////						speed->time = HAL_GetTick();
////					  break;
////					case 's':
////						message[0] = 's';
////					  speed->y = 0;//y
////						speed->x = 0;
////						speed->r = 0;
////						speed->cal_speed = 1;
////						speed->time = HAL_GetTick();
////					  break;
////					default:
////						message[0] = 'e';
////					  speed->y = 0;//y
////						speed->x = 0;
////						speed->r = 0;
////						speed->cal_speed = 1;
////						speed->time = HAL_GetTick();
////				}
//				//HAL_UART_Transmit_DMA(&UART_PORT, (uint8_t *)message, 1);
//				printf("aaa");
//			bt_cnt = 0;
//			break;
//			}
//			bt_cnt = 0;
//		default:
//			bt_cnt = 0;		
//	}
//	
//	#if OFFLINE_DECT == 1
//	if(speed->time+TIMEOUT_TH <=  HAL_GetTick())
//	{
//		speed->y = 0;
//		speed->x = 0;
//		speed->r = 0;
//		speed->cal_speed = 1;
//		speed->time = HAL_GetTick();
//	}
//	#endif
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &UART_PORT)
	{
			switch(usart_cnt)
		{
			case 1:
			if(0x31 == usartRxbuf[0])
			{
				usart_cnt = 2;
			}
			else
			{
				usart_cnt = 0;
			}
			break;
			case 3:
				usart_cnt = 4;
			break;
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	
}

/* USER CODE BEGIN 1 */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}
/* USER CODE END 1 */
