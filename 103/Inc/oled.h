 //////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//�о�԰����
//���̵�ַ��http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  �� �� ��   : main.c
//  �� �� ��   : v2.0
//  ��    ��   : Evk123
//  ��������   : 2014-0101
//  ����޸�   : 
//  ��������   : 0.69��OLED �ӿ���ʾ����(STM32F103ZEϵ��IIC)
//              ˵��: 
//              ----------------------------------------------------------------
//              GND   ��Դ��
//              VCC   ��5V��3.3v��Դ
//              SCL   ��PB13��SCL��
//              SDA   ��PB15��SDA�� 
//              RES   ��PB11 �������4��iic�ӿ�����ſ��Բ���
//              ----------------------------------------------------------------
//Copyright(C) �о�԰����2014/3/16
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////
#ifndef __OLED_H
#define __OLED_H			  	 
//#include "sys.h"
//#include "stdlib.h"	 

#define HIIC hi2c1
#define u8 uint8_t
#define u32 uint32_t
   	
#define OLED_MODE 0
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED IIC�˿ڶ���----------------  					   
/*
#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_6)//CLK
#define OLED_SCLK_Set() GPIO_SetBits(GPIOB,GPIO_Pin_6)

#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_7)//DIN
#define OLED_SDIN_Set() GPIO_SetBits(GPIOB,GPIO_Pin_7)

//#define OLED_RST_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_11)//RES
//#define OLED_RST_Set() GPIO_SetBits(GPIOB,GPIO_Pin_11)*/
 		     
#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

extern I2C_HandleTypeDef HIIC;

//OLED�����ú���
//void OLED_WR_Byte(unsigned dat,unsigned cmd);  
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
uint8_t OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y, u8 *p,u8 Char_Size);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void fill_picture(unsigned char fill_Data);

/***********************Delay****************************************/
__STATIC_INLINE void Delay_50ms(unsigned int Del_50ms)
{
	HAL_Delay(50*Del_50ms);
}

__STATIC_INLINE void Delay_1ms(unsigned int Del_1ms)
{
	HAL_Delay(Del_1ms);
}

// IIC Write Command
/**********************************************/
__STATIC_INLINE uint8_t Write_IIC_Command(unsigned char IIC_Command)
{
	uint8_t txbuf[2] = {0x00,IIC_Command};	
	if(HAL_OK == HAL_I2C_Master_Transmit(&HIIC,0X78,txbuf,2,10))
		return 0;
	else 
		return 0xff;
}
/**********************************************
// IIC Write Data
**********************************************/
__STATIC_INLINE uint8_t Write_IIC_Data(unsigned char IIC_Data)
{
	uint8_t txbuf[2] = {0x40,IIC_Data};	
	if(HAL_OK == HAL_I2C_Master_Transmit(&HIIC,0X78,txbuf,2,10))
		return 0;
	else 
		return 0xff;
}

__STATIC_INLINE uint8_t OLED_WR_Byte(unsigned dat,unsigned cmd)
{
	if(cmd)
		return Write_IIC_Data(dat);
	else 
		return Write_IIC_Command(dat);
}

#endif
