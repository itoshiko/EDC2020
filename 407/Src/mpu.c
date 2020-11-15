#include "main.h"
#include "stm32f4xx_hal.h"
#include "config.h"
#include "stdio.h"
#include "mpu.h"

extern UART_HandleTypeDef UART_PORT3;

uint8_t mpu_cnt = 0;
uint8_t mpuRxbuf[22]={0};

void mpu_task(mpuData * data)
{
	/*state mechime : 
	0:free ,set dma
	1:wait to recieved start frame 
	2:start frame recieved,set dma for body
	3:waiting for body(dma)
	4:data recieved complete*/	
	switch(mpu_cnt)
	{
		case 0:
			mpuRxbuf[0] = 0;
			HAL_UART_Receive_DMA(&UART_PORT3,mpuRxbuf,1);
			mpu_cnt = 1;
		break;
		case 1:
			break;
		case 2:
			HAL_UART_Receive_DMA(&UART_PORT3,&mpuRxbuf[1],21);
			mpu_cnt = 3;
		break;
			case 3:
		break;
		case 4:
			if(0x52 == mpuRxbuf[1])
			{
        data->an_vel = (float)((mpuRxbuf[7] << 8) | mpuRxbuf[6]) / 32768.0 * 2000.0;
				data->an = (float)((mpuRxbuf[18] << 8) | mpuRxbuf[17]) / 32768.0 * 180.0;
				mpu_cnt = 0;
				break;
			}
			mpu_cnt = 0;
		default:
			mpu_cnt = 0;		
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &UART_PORT3)
	{
			switch(mpu_cnt)
		{
			case 1:
			if(0x55 == mpuRxbuf[0])
			{
				mpu_cnt = 2;
			}
			else
			{
				mpu_cnt = 0;
			}
			break;
			case 3:
				mpu_cnt = 4;
			break;
		}
	}
}