#include "main.h"
#include "stm32f1xx_hal.h"
#include "led.h"
#include "motor.h"
#include "ps2.h"
//#include "searchpath.h"

//extern mt_ctrltype MT_CTRL;
//extern pidtype MT_MTS[4];
//extern SPI_HandleTypeDef SPIPORT;
//extern int16_t SPEED_XYR[3];
//extern uint8_t CAL_SPEED;
uint8_t ps2Txbuf[BUFFSIZE]={0x01,0x42,0xff,0xff,0xff,0xff,0xff,0xff,0xff},ps2Rxbuf[BUFFSIZE]={0};
uint8_t ps2_busy = 0;
uint8_t lastmode = 0;

static void uDelay(uint32_t t)
{
	uint32_t i;
	for(i=0;i<t;++i)
		__nop();
}

void Ps2_task(mt_ctrltype *ctrl,pidtype *mt,speed3axistype *speed)
{
	uint8_t i,j;
	uint8_t tmp,send;
	static uint32_t time_next = 0;
	uint32_t time_curr = HAL_GetTick();
	if(time_curr <= time_next)
		return;
	if(ps2_busy == 0)
	{
		/*clean buf*/
		for(i=0;i<2;++i)
			ps2Rxbuf[i] = 0;
		
		/*read data from ps2*/
		//HAL_SPI_TransmitReceive_DMA(&SPIPORT,Txbuf,Rxbuf,BUFFSIZE);
		LL_GPIO_ResetOutputPin(ENBANK, ENPORT);
		LL_GPIO_ResetOutputPin(ENBANK, ENPORT);
		uDelay(1);
		for(i=0;i<BUFFSIZE;++i)
		{
			tmp = 0;
			send = ps2Txbuf[i];
			if((send & 0x01) == 0)
				{
					LL_GPIO_ResetOutputPin(SPIBANK, CMD);
				}
				else
				{
					LL_GPIO_SetOutputPin(SPIBANK, CMD);
				}
				for(j=0;j<8;++j)
			{
				tmp >>= 1;
				send >>= 1;
				LL_GPIO_ResetOutputPin(SPIBANK, SCK);
				uDelay(1);
				if(GPIO_PIN_SET == HAL_GPIO_ReadPin(SPIBANK,DAT))
					tmp |= 0x80;
				uDelay(71);
				LL_GPIO_SetOutputPin(SPIBANK, SCK);
				if((send & 0x01) == 0)
				{
					LL_GPIO_ResetOutputPin(SPIBANK, CMD);
				}
				else
				{
					LL_GPIO_SetOutputPin(SPIBANK, CMD);
				}
				
				uDelay(74);
			}
			ps2Rxbuf[i] = tmp;
			uDelay(70);
		}
		LL_GPIO_SetOutputPin(ENBANK, ENPORT);
		
		/*slove ps2 data*/
		if((ps2Rxbuf[1]==0x73)&&(ps2Rxbuf[2]==0x5a))
		{
			//mode0
			/*if(lastmode != 0)
			{
				Motor_PID_Enable(ctrl,mt);
				lastmode = 0;
			}*/
			speed->y=(ps2Rxbuf[7]<<3)-1023;//y
			speed->x=-(ps2Rxbuf[8]<<3)+1023;
			speed->r=(ps2Rxbuf[5]<<3)-1023;
			
			speed->y = (speed->y <= JOY_TH && speed->y >= -JOY_TH)?0:speed->y;
			speed->x = (speed->x <= JOY_TH && speed->x >= -JOY_TH)?0:speed->x;
			speed->r = (speed->r <= JOY_TH && speed->r >= -JOY_TH)?0:speed->r;
			
			speed->cal_speed = 1;
			//Motor_unlock(&MT_CTRL);
			//Set_blink(1,6);//ms
		}
		else if((ps2Rxbuf[1]==0x41)&&(ps2Rxbuf[2]==0x5a))
		{
			//mode1
			//Searchpath(SPEED_XYR,mt);
			//CAL_SPEED = 1;
		}
		/*else 
		{
			Motor_lock(&MT_CTRL);
		}*/
		time_next = time_curr + 10;
		//ps2_busy = 1;
	}
}

