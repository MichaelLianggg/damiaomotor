#ifndef __LCD_H
#define __LCD_H


#include "main.h"
#include "spi.h"


#define USE_ANALOG_SPI 0	// Select software SPI or hardware SPI: 0 = hardware SPI, 1 = software SPI
#define USE_HORIZONTAL 2  // Set display orientation: 0/1 = portrait, 2/3 = landscape

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 280

#else
#define LCD_W 280
#define LCD_H 240
#endif

 
//-----------------LCD pin definitions---------------- 
#if USE_ANALOG_SPI
#define LCD_SCLK_Clr() HAL_GPIO_WritePin(LCD_SCK_GPIO_Port,LCD_SCK_Pin, GPIO_PIN_RESET)//SCL=SCLK
#define LCD_SCLK_Set() HAL_GPIO_WritePin(LCD_SCK_GPIO_Port,LCD_SCK_Pin, GPIO_PIN_SET)

#define LCD_MOSI_Clr() HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_SDA_Pin, GPIO_PIN_RESET)//SDA=MOSI
#define LCD_MOSI_Set() HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_SDA_Pin, GPIO_PIN_SET)
#endif

#define LCD_RES_Clr()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin, GPIO_PIN_RESET)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin, GPIO_PIN_SET)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin, GPIO_PIN_RESET)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin, GPIO_PIN_SET)
 		     
#define LCD_CS_Clr()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin, GPIO_PIN_RESET)//CS
#define LCD_CS_Set()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin, GPIO_PIN_SET)

#define LCD_BLK_Clr(x)  TIM1->CCR1=x//HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin, GPIO_PIN_RESET)//BLK
#define LCD_BLK_Set(x)  TIM1->CCR1=x//HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin, GPIO_PIN_SET)

void LCD_Init(void); 						// LCD initialization
void LCD_Writ_Bus(uint8_t dat); 	// Emulate SPI timing
void LCD_WR_DATA8(uint8_t dat); 	// Write one byte
void LCD_WR_DATA(uint16_t dat); 	// Write two bytes
void LCD_WR_REG(uint8_t dat); 		// Write a command
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2); // Set address/window

void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color); // Fill a rectangular area with color
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color); // Draw a point at the specified position
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color); // Draw a line at the specified position
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color); // Draw a rectangle at the specified position
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color); // Draw a circle at the specified position

void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); 		// Display a Chinese character string
void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // Display a single 12x12 Chinese character
void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // Display a single 16x16 Chinese character
void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // Display a single 24x24 Chinese character
void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // Display a single 32x32 Chinese character

void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); 			// Display a character
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // Display a string
uint32_t mypow(uint8_t m,uint8_t n); // Power function
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey); // Display an integer
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey); 	// Display a signed float
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey); 	// Display a positive float
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[]); // Display an image


// Pen colors

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 // brown
#define BRRED 			     0XFC07 // brown-red
#define GRAY  			     0X8430 // gray
#define DARKBLUE      	 0X01CF	// dark blue
#define LIGHTBLUE     	 0X7D7C	// light blue
#define GRAYBLUE       	 0X5458 // gray-blue
#define LIGHTGREEN     	 0X841F // light green
#define LGRAY 			     0XC618 // light gray (panel background)
#define LGRAYBLUE       0XA651 // light gray-blue (middle layer)
#define LBBLUE           0X2B12 // light brown-blue (selection inverse)
#endif





