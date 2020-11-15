#ifndef __PS2_H__
#define __PS2_H__

#define ENBANK GPIOC
#define ENPORT LL_GPIO_PIN_2
#define MT_CTRL common_mt_ctrl
#define MT_MTS motors
#define SPIPORT hspi1
#define SPEED_XYR speed_xyr
#define CAL_SPEED cal_speed
#define BUFFSIZE 9
#define SPIBANK GPIOA
#define SCK LL_GPIO_PIN_5
#define CMD LL_GPIO_PIN_7
#define DAT GPIO_PIN_6
#define JOY_TH 10

void Ps2_task(mt_ctrltype *ctrl,pidtype *mt,speed3axistype *speed);

#endif
