#include "lcd.h"
#include "lcdfont.h"

/**
************************************************************************
* @brief:       LCD_Writ_Bus: low-level write helper for LCD
* @param:       dat - byte to send
* @retval:      void
* @details:     Sends bytes to the LCD. Uses bit-banged GPIO when USE_ANALOG_SPI is enabled
*               or hardware SPI (HAL_SPI_Transmit) when disabled.
*               - When USE_ANALOG_SPI is enabled, toggles SCLK/MOSI/CS via GPIO to send 8 bits.
*               - When disabled, uses HAL_SPI_Transmit to send one byte via hardware SPI.
************************************************************************
**/
void LCD_Writ_Bus(uint8_t dat) 
{	
	LCD_CS_Clr();
#if USE_ANALOG_SPI
	for(uint8_t i=0;i<8;i++) {			  
		LCD_SCLK_Clr();
		if(dat&0x80) {
		   LCD_MOSI_Set();
		} else {
		   LCD_MOSI_Clr();
		}
		LCD_SCLK_Set();
		dat<<=1;
	}
#else
	HAL_SPI_Transmit(&hspi1, &dat, 1, 0xffff);
#endif	
  LCD_CS_Set();	
}
/**
************************************************************************
* @brief:       LCD_WR_DATA8: write 8-bit data to LCD
* @param:       dat - 8-bit data to write
* @retval:      void
* @details:     Uses LCD_Writ_Bus to send an 8-bit data value to the LCD.
************************************************************************
**/
void LCD_WR_DATA8(uint8_t dat)
{
	LCD_Writ_Bus(dat);
}
/**
************************************************************************
* @brief:       LCD_WR_DATA: write 16-bit data to LCD
* @param:       dat - 16-bit data to write
* @retval:      void
* @details:     Sends a 16-bit value by writing the high byte then the low byte via LCD_Writ_Bus.
************************************************************************
**/
void LCD_WR_DATA(uint16_t dat)
{
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}
/**
************************************************************************
* @brief:       LCD_WR_REG: write a command register to LCD
* @param:       dat - command byte to write
* @retval:      void
* @details:     Uses LCD_Writ_Bus to send a command byte to the LCD (DC pin is toggled accordingly).
************************************************************************
**/
void LCD_WR_REG(uint8_t dat)
{
	LCD_DC_Clr();//д����
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//д����
}
/**
************************************************************************
* @brief:       LCD_Address_Set: set the LCD display address/window
* @param:       x1, y1, x2, y2 - coordinates describing the display window
* @retval:      void
* @details:     Sends the appropriate register/parameter sequence to the LCD to set the active
*               drawing window where subsequent pixel data will be written.
************************************************************************
**/
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
/**
************************************************************************
* @brief:       LCD_Fill: fill a rectangular area on the LCD with a color
* @param:       xsta, ysta, xend, yend - rectangle coordinates
*               color - fill color
* @retval:      void
* @details:     Sets the address window, then writes pixel color repeatedly to fill the area.
************************************************************************
**/
		LCD_WR_REG(0x2c);//������д
	/**
	************************************************************************
	* @brief:       LCD_DrawPoint: draw a pixel at specified coordinates
	* @param:       x, y - coordinates
	*               color - pixel color
	* @retval:      void
	* @details:     Sets the address to the single pixel and writes the color value.
	************************************************************************
	**/
		LCD_WR_REG(0x2c);//������д
	/**
	************************************************************************
	* @brief:       LCD_DrawLine: draw a line on the LCD
	* @param:       x1, y1 - start coordinates
	*               x2, y2 - end coordinates
	*               color - line color
	* @retval:      void
	* @details:     Implements a Bresenham-like line drawing algorithm and writes pixels using LCD_DrawPoint.
	************************************************************************
	**/
	/**
	************************************************************************
	* @brief:       LCD_DrawRectangle: draw a rectangle
	* @param:       x1, y1 - top-left corner
	*               x2, y2 - bottom-right corner
	*               color - rectangle color
	* @retval:      void
	* @details:     Draws rectangle edges using LCD_DrawLine.
	************************************************************************
	**/
	/**
	************************************************************************
	* @brief:       Draw_Circle: draw a circle on the LCD
	* @param:       x0, y0 - center coordinates
	*               r - radius
	*               color - circle color
	* @retval:      void
	* @details:     Uses midpoint circle algorithm to draw a circle centered at (x0, y0) with radius r.
	************************************************************************
	**/
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{          
	uint16_t i,j; 
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//������ʾ��Χ
	for(i=ysta;i<yend;i++)
	{													   	 	
		for(j=xsta;j<xend;j++)
		{
			LCD_WR_DATA(color);
		}
	} 					  	    
}
/**
************************************************************************
* @brief:      	LCD_DrawPoint: ��LCDָ��λ�û���
* @param:      	x, y - �������
*              	color - �����ɫ
* @retval:     	void
* @details:    	ͨ������LCD_Address_Set��������LCD��ʾ����ĵ�ַ��Χ��Ȼ����ָ��λ�û��㣬��ɫ��color����ָ����
************************************************************************
**/
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_Address_Set(x,y,x,y);//���ù��λ�� 
	LCD_WR_DATA(color);
} 
/**
************************************************************************
* @brief:      	LCD_DrawLine: ��LCD�ϻ���
* @param:      	x1, y1 - �ߵ���ʼ����
*              	x2, y2 - �ߵĽ�������
*              	color - �ߵ���ɫ
* @retval:     	void
* @details:    	ͨ����������������ѡ��������������ᣬ����ʼ���굽����������㻭�ߣ���ɫ��color����ָ����
************************************************************************
**/
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //������������ 
	delta_y=y2-y1;
	uRow=x1;//�����������
	uCol=y1;
	if(delta_x>0)incx=1; //���õ������� 
	else if (delta_x==0)incx=0;//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//ˮƽ�� 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//����
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}
/**
************************************************************************
* @brief:      	LCD_DrawRectangle: ��LCD�ϻ�����
* @param:      	x1, y1 - ���ε����Ͻ�����
*              	x2, y2 - ���ε����½�����
*              	color - ���ε���ɫ
* @retval:     	void
* @details:    	ͨ������LCD_DrawLine�����������ε������ߣ���ɫ��color����ָ����
************************************************************************
**/
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}
/**
************************************************************************
* @brief:      	Draw_Circle: ��LCD�ϻ�Բ
* @param:      	x0, y0 - Բ������
*              	r - Բ�İ뾶
*              	color - Բ����ɫ
* @retval:     	void
* @details:    	ʹ���е㻭Բ������Բ��(x0, y0)Ϊ���ģ��뾶Ϊr����LCD�ϻ���Բ��
*              	��ɫ��color����ָ����
************************************************************************
**/
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		/**
		************************************************************************
		* @brief:       LCD_ShowChinese: display Chinese character string on the LCD
		* @param:       x, y - start coordinates and display anchor
		*               s - pointer to GB2312/GBK encoded character data (each Chinese char usually 2 bytes)
		*               fc - foreground color
		*               bc - background color
		*               sizey - font size (supports 12x12, 16x16, 24x24, 32x32)
		*               mode - display mode: 1 = foreground only, 0 = background fill
		* @retval:      void
		* @details:     Chooses the font rendering routine based on sizey and renders characters accordingly.
		************************************************************************
		**/
*              	bc - ������ɫ
*              	sizey - ���������С��֧��12x12��16x16��24x24��32x32
*              	mode - ��ʾģʽ��1��ʾ��ɫ��0��ʾ������ʾ
* @retval:     	void
* @details:    	����sizeyѡ�������С����LCD����ʾ�����ַ���������ѡ����ʾģʽ��
************************************************************************
**/
void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	while(*s!=0)
	{
		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
		else return;
		s+=2;
		x+=sizey;
	}
}
/**
************************************************************************
* @brief:       LCD_ShowChinese12x12: display 12x12 Chinese character
* @param:       x, y - start coordinates
*               s - pointer to character code (2 bytes per Chinese char)
*               fc - foreground color
*               bc - background color
*               sizey - font size (12 supported)
*               mode - display mode: 1 = foreground only, 0 = background fill
* @retval:      void
* @details:     Renders a single 12x12 Chinese character from font table.
************************************************************************
**/
void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//������Ŀ
	uint16_t TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//ͳ�ƺ�����Ŀ
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//�ǵ��ӷ�ʽ
					{
						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//���ӷ�ʽ
					{
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
} 
/**
************************************************************************
* @brief:      	LCD_ShowChinese16x16: ��LCD����ʾ16x16����
* @param:      	x, y - ��ʼ���꣬��ʾ���ֵ����Ͻ�����
*              	s - Ҫ��ʾ�ĺ��֣�ÿ������ռ�����ֽ�
*              	fc - ������ɫ
*              	bc - ������ɫ
*              	sizey - ���������С��֧��16x16
*              	mode - ��ʾģʽ��1��ʾ��ɫ��0��ʾ������ʾ
* @retval:     	void
* @details:    	��LCD����ʾ16x16���֣�����ѡ����ʾģʽ��
************************************************************************
**/
void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//������Ŀ
	uint16_t TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	uint16_t x0=x;
  TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//ͳ�ƺ�����Ŀ
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//�ǵ��ӷ�ʽ
					{
						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//���ӷ�ʽ
					{
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
} 
/**
************************************************************************
* @brief:      	LCD_ShowChinese24x24: ��LCD����ʾ24x24����
* @param:      	x, y - ��ʼ���꣬��ʾ���ֵ����Ͻ�����
*              	s - Ҫ��ʾ�ĺ��֣�ÿ������ռ�����ֽ�
*              	fc - ������ɫ
*              	bc - ������ɫ
*              	sizey - ���������С��֧��24x24
*              	mode - ��ʾģʽ��1��ʾ��ɫ��0��ʾ������ʾ
* @retval:     	void
* @details:    	��LCD����ʾ24x24���֣�����ѡ����ʾģʽ��
************************************************************************
**/
void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//������Ŀ
	uint16_t TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//ͳ�ƺ�����Ŀ
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//�ǵ��ӷ�ʽ
					{
						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//���ӷ�ʽ
					{
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
} 
/**
************************************************************************
* @brief:      	LCD_ShowChinese32x32: ��LCD����ʾ32x32����
* @param:      	x, y - ��ʼ���꣬��ʾ���ֵ����Ͻ�����
*              	s - Ҫ��ʾ�ĺ��֣�ÿ������ռ�����ֽ�
*              	fc - ������ɫ
*              	bc - ������ɫ
*              	sizey - ���������С��֧��32x32
*              	mode - ��ʾģʽ��1��ʾ��ɫ��0��ʾ������ʾ
* @retval:     	void
* @details:    	��LCD����ʾ32x32���֣�����ѡ����ʾģʽ��
************************************************************************
**/
void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//������Ŀ
	uint16_t TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//ͳ�ƺ�����Ŀ
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//�ǵ��ӷ�ʽ
					{
						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//���ӷ�ʽ
					{
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
}
/**
************************************************************************
* @brief:      	LCD_ShowChar: ��LCD����ʾ�����ַ�
* @param:      	x, y - ��ʼ���꣬��ʾ�ַ������Ͻ�����
*              	num - Ҫ��ʾ���ַ���ASCII��ֵ
*              	fc - ������ɫ
*              	bc - ������ɫ
*              	sizey - �����С��֧��12x6, 16x8, 24x12, 32x16
*              	mode - ��ʾģʽ��1��ʾ��ɫ��0��ʾ������ʾ
* @retval:     	void
* @details:    	��LCD����ʾ�����ַ�������ѡ����ʾģʽ�Ͳ�ͬ�������С��
************************************************************************
**/
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t temp,sizex,t,m=0;
	uint16_t i,TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	uint16_t x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //�õ�ƫ�ƺ��ֵ
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //���ù��λ�� 
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //����6x12����
		else if(sizey==16)temp=ascii_1608[num][i];		 //����8x16����
		else if(sizey==24)temp=ascii_2412[num][i];		 //����12x24����
		else if(sizey==32)temp=ascii_3216[num][i];		 //����16x32����
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//�ǵ���ģʽ
			{
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//����ģʽ
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//��һ����
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}
/**
************************************************************************
* @brief:      	LCD_ShowString: ��LCD����ʾ�ַ���
* @param:      	x, y - ��ʼ���꣬��ʾ�ַ��������Ͻ�����
*              	p - Ҫ��ʾ���ַ�������'\0'��β
*              	fc - ������ɫ
*              	bc - ������ɫ
*              	sizey - �����С��֧��12x6, 16x8, 24x12, 32x16
*              	mode - ��ʾģʽ��1��ʾ��ɫ��0��ʾ������ʾ
* @retval:     	void
* @details:    	��LCD����ʾ�ַ���������ѡ����ʾģʽ�Ͳ�ͬ�������С��
************************************************************************
**/
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}
/**
************************************************************************
* @brief:      	mypow: ����m��n�η�
* @param:      	m - ����
*              	n - ָ��
* @retval:     	uint32_t - m��n�η�
* @details:    	����m��n�η������ؽ����
************************************************************************
**/
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;
	return result;
}
/**
************************************************************************
* @brief:      	LCD_ShowIntNum: ��LCD����ʾ��������
* @param:      	x - x����
*              	y - y����
*              	num - Ҫ��ʾ������
*              	len - ������ʾ��λ��
*              	fc - ������ɫ
*              	bc - ������ɫ
*              	sizey - �����С
* @retval:     	void
* @details:    	��LCD����ʾ�������֣�֧��������ʾ��λ����������ɫ��������ɫ�������С��
************************************************************************
**/
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	uint8_t sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 
/**
************************************************************************
* @brief:      	LCD_ShowFloatNum: ��LCD����ʾ��ʽ���ĸ�������֧�ָ���
* @param:      	x - x����
*              	y - y����
*              	num - Ҫ��ʾ�ĸ�����
*              	len - ����λ��
*              	decimal - С��λ��
*              	fc - �ֵ���ɫ
*              	bc - ������ɫ
*              	sizey - �����С
* @retval:     	void
* @details:    	��LCD����ʾ��ʽ���ĸ�������֧����������λ����С��λ����������ɫ��������ɫ�������С��
************************************************************************
**/
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
		int16_t num_int;
		uint8_t t, temp, sizex;
    sizex = sizey / 2;
    num_int = num * mypow(10, decimal);

    if (num < 0)
    {
        LCD_ShowChar(x, y, '-', fc, bc, sizey, 0);
        num_int = -num_int;
        x += sizex;
        len++;
    }
    else
    {
        LCD_ShowChar(x, y, ' ', fc, bc, sizey, 0);
        num_int = num_int;
        x += sizex;
        len++;
    }

    // �ڸ�������ʱˢ����ʾ��λ��
    LCD_Fill(x, y, x + len * sizex + decimal + 1, y + sizey + 1, bc);

    for (t = 0; t < len; t++)
    {
        if (t == (len - decimal))
        {
            LCD_ShowChar(x + (len - decimal) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        temp = ((num_int / mypow(10, len - t - 1)) % 10) + '0';
        LCD_ShowChar(x + t * sizex, y, temp, fc, bc, sizey, 0);
    }
}
/**
************************************************************************
* @brief:      	LCD_ShowFloatNum1: ��LCD����ʾ��ʽ���ĸ���������֧�ָ���
* @param:      	x - x����
*              	y - y����
*              	num - Ҫ��ʾ�ĸ�����
*              	len - ����λ��
*              	decimal - С��λ��
*              	fc - �ֵ���ɫ
*              	bc - ������ɫ
*              	sizey - �����С
* @retval:     	void
* @details:    	��LCD����ʾ��ʽ���ĸ�������֧����������λ����С��λ����������ɫ��������ɫ�������С��
************************************************************************
**/
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
		int16_t num_int;
		uint8_t t, temp, sizex;
    sizex = sizey / 2;
    num_int = num * mypow(10, decimal);


		num_int = num_int;
		x += sizex;
		len++;

    // �ڸ�������ʱˢ����ʾ��λ��
    LCD_Fill(x, y, x + len * sizex + decimal + 1, y + sizey + 1, bc);

    for (t = 0; t < len; t++)
    {
        if (t == (len - decimal))
        {
            LCD_ShowChar(x + (len - decimal) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        temp = ((num_int / mypow(10, len - t - 1)) % 10) + '0';
        LCD_ShowChar(x + t * sizex, y, temp, fc, bc, sizey, 0);
    }
}
/**
************************************************************************
* @brief:      	LCD_ShowFloatNum1: ��LCD����ʾ��ʽ���ĸ���������֧�ָ���
* @param:      	x - x����
*              	y - y����
*              	num - Ҫ��ʾ�ĸ�����
*              	len - ����λ��
*              	decimal - С��λ��
*              	fc - �ֵ���ɫ
*              	bc - ������ɫ
*              	sizey - �����С
* @retval:     	void
* @details:    	��LCD����ʾ��ʽ���ĸ�������֧����������λ����С��λ����������ɫ��������ɫ�������С��
************************************************************************
**/
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[])
{
	uint16_t i,j;
	uint32_t k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}
/**
************************************************************************
* @brief:      	LCD_Init: LCD��ʼ������
* @param:      	void
* @details:    	ִ��LCD�ĳ�ʼ�����̣�������λ��������ơ��Ĵ������õ�
* @retval:     	void
************************************************************************
**/
void LCD_Init(void)
{
	
	LCD_RES_Clr();//��λ
	HAL_Delay(100);
	LCD_RES_Set();
	HAL_Delay(100);
	
	LCD_BLK_Set(900);//�򿪱���
  HAL_Delay(100);
	
	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11); //Sleep out 
	HAL_Delay(120);              //Delay 120ms 
	//************* Start Initial Sequence **********// 
	LCD_WR_REG(0x36);
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
	else LCD_WR_DATA8(0xA0);

	LCD_WR_REG(0x3A);			
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0xB2);			
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C); 
	LCD_WR_DATA8(0x00); 
	LCD_WR_DATA8(0x33); 
	LCD_WR_DATA8(0x33); 			

	LCD_WR_REG(0xB7);			
	LCD_WR_DATA8(0x35);

	LCD_WR_REG(0xBB);			
	LCD_WR_DATA8(0x32); //Vcom=1.35V
					
	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);			
	LCD_WR_DATA8(0x15); //GVDD=4.8V  ��ɫ���
				
	LCD_WR_REG(0xC4);			
	LCD_WR_DATA8(0x20); //VDV, 0x20:0v

	LCD_WR_REG(0xC6);			
	LCD_WR_DATA8(0x0F); //0x0F:60Hz        	

	LCD_WR_REG(0xD0);			
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1); 

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xD0);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x48);   
	LCD_WR_DATA8(0x17);   
	LCD_WR_DATA8(0x14);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x34);   

	LCD_WR_REG(0xE1);     
	LCD_WR_DATA8(0xD0);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x48);   
	LCD_WR_DATA8(0x17);   
	LCD_WR_DATA8(0x14);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x34);
	LCD_WR_REG(0x21); 

	LCD_WR_REG(0x29);
}

