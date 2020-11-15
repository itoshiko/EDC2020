#include "main.h"
#include "stm32f1xx_hal.h"
#include "servo.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

void user_Servo_Init(void)
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
}
