#include "LCD.h"

void gui_lcd(unsigned char d)
{
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port,LCD_D4_Pin,(GPIO_PinState)((d&16)>>4));
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port,LCD_D5_Pin,(GPIO_PinState)((d&32)>>5));
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port,LCD_D6_Pin,(GPIO_PinState)((d&64)>>6));
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port,LCD_D7_Pin,(GPIO_PinState)((d&128)>>7));
	
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port,LCD_D4_Pin,(GPIO_PinState)((d&1)>>0));
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port,LCD_D5_Pin,(GPIO_PinState)((d&2)>>1));
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port,LCD_D6_Pin,(GPIO_PinState)((d&4)>>2));
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port,LCD_D7_Pin,(GPIO_PinState)((d&8)>>3));
	
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,GPIO_PIN_RESET);
	
}	
void lenh_lcd(unsigned char d)
{
	//delay_lcd(200);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,GPIO_PIN_RESET);
	gui_lcd(d);	
}
void ma_lcd(unsigned char d)
{
	//delay_lcd(200);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,GPIO_PIN_SET);
	gui_lcd(d);	
}
void string_lcd(unsigned char *s)
{
	while(*s)
	{
		ma_lcd(*(s++));	
	}	
}	
void kl(void)
{
	HAL_Delay(30);
	//delay_lcd(30000);
	lenh_lcd(0x2);
	lenh_lcd(0x28);
	lenh_lcd(0xc);
	lenh_lcd(0x1);	
}	
