#include "LCD.h"

/*屏幕引脚
LCD-PWR 背光线
LCD-RST 复位
LCD-DC DC引脚用于区分发送到显示屏的数据类型：
命令模式：当DC引脚为低电平时，表示发送的是命令。命令用于控制显示屏的行为，如清屏、设置显示模式等。
数据模式：当DC引脚为高电平时，表示发送的是数据。数据用于在显示屏上显示具体的内容，如文字、图像等
LCD-MISO 发送
LCD-SCK 时钟
*/

#define LCD_TOTAL_BUF_SIZE	(240*240*2)
#define LCD_Buf_Size 1152
static uint8_t lcd_buf[LCD_Buf_Size];

uint16_t	POINT_COLOR = WHITE;	//画笔颜色	默认为黑色
uint16_t	BACK_COLOR 	= BLACK;	//背景颜色	默认为白色

/**
 * @brief	LCD初始化
 *
 * @param   void
 *
 * @return  void
 */
void LCD_Init(void)
{
    LCD_RES_SET() ;
    HAL_Delay(12);
    LCD_RES_RESET();
	
    HAL_Delay(12);
    /* Sleep Out */
    LCD_Write_Cmd(0x11);
    /* wait for power stability */
    HAL_Delay(12);

    /* Memory Data Access Control */
    LCD_Write_Cmd(0x36);
    LCD_Write_Data(0x00);

    /* RGB 5-6-5-bit  */
    LCD_Write_Cmd(0x3A);
    LCD_Write_Data(0x65);

    /* Porch Setting */
    LCD_Write_Cmd(0xB2);
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x00);
    LCD_Write_Data(0x33);
    LCD_Write_Data(0x33);


    /*  Gate Control */
    LCD_Write_Cmd(0xB7);
    LCD_Write_Data(0x72);

    /* VCOM Setting */
    LCD_Write_Cmd(0xBB);
    LCD_Write_Data(0x3D);   //Vcom=1.625V

    /* LCM Control */
    LCD_Write_Cmd(0xC0);
    LCD_Write_Data(0x2C);

    /* VDV and VRH Command Enable */
    LCD_Write_Cmd(0xC2);
    LCD_Write_Data(0x01);

    /* VRH Set */
    LCD_Write_Cmd(0xC3);
    LCD_Write_Data(0x19);

    /* VDV Set */
    LCD_Write_Cmd(0xC4);
    LCD_Write_Data(0x20);

    /* Frame Rate Control in Normal Mode */
    LCD_Write_Cmd(0xC6);
    LCD_Write_Data(0x0F);	//60MHZ

    /* Power Control 1 */
    LCD_Write_Cmd(0xD0);
    LCD_Write_Data(0xA4);
    LCD_Write_Data(0xA1);

    /* Positive Voltage Gamma Control */
    LCD_Write_Cmd(0xE0);
    LCD_Write_Data(0xD0);
    LCD_Write_Data(0x04);
    LCD_Write_Data(0x0D);
    LCD_Write_Data(0x11);
    LCD_Write_Data(0x13);
    LCD_Write_Data(0x2B);
    LCD_Write_Data(0x3F);
    LCD_Write_Data(0x54);
    LCD_Write_Data(0x4C);
    LCD_Write_Data(0x18);
    LCD_Write_Data(0x0D);
    LCD_Write_Data(0x0B);
    LCD_Write_Data(0x1F);
    LCD_Write_Data(0x23);

    /* Negative Voltage Gamma Control */
    LCD_Write_Cmd(0xE1);
    LCD_Write_Data(0xD0);
    LCD_Write_Data(0x04);
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x11);
    LCD_Write_Data(0x13);
    LCD_Write_Data(0x2C);
    LCD_Write_Data(0x3F);
    LCD_Write_Data(0x44);
    LCD_Write_Data(0x51);
    LCD_Write_Data(0x2F);
    LCD_Write_Data(0x1F);
    LCD_Write_Data(0x1F);
    LCD_Write_Data(0x20);
    LCD_Write_Data(0x23);

    /* Display Inversion On */
    LCD_Write_Cmd(0x21);

    LCD_Write_Cmd(0x29);

    LCD_Address_Set(0, 0, LCD_Width - 1, LCD_Height - 1);

    LCD_Clear(WHITE);

    /* Display on */

}


/**
 * @brief	写命令到LCD
 *
 * @param   cmd		需要发送的命令
 *
 * @return  void
 */
static void LCD_Write_Cmd(uint8_t cmd)
{
    LCD_WR_CMD();

    LCD_SPI_Send(&cmd, 1);
}

/**
 * @brief	写数据到LCD
 *
 * @param   cmd		需要发送的数据
 *
 * @return  void
 */
static void LCD_Write_Data(uint8_t data)
{
    LCD_WR_DATA();

    LCD_SPI_Send(&data, 1);
}



/**
 * @brief	LCD底层SPI发送数据函数
 *
 * @param   data	数据的起始地址
 * @param   size	发送数据大小
 *
 * @return  void
 */
static void LCD_SPI_Send(uint8_t *data, uint16_t size)
{
	SPI1_WriteByte(data, size);
}
uint8_t SPI1_WriteByte(uint8_t* data, uint16_t size)
{
	return HAL_SPI_Transmit(&hspi1, data, size, 0xff);
}



/******************************************************
 * 函数名：PutChinese21
 * 描述  ：显示单个汉字字符串
 * 输入  : pos: 0~(319-16)
 *         Ypos: 0~(239-16)
 *				 str: 中文字符串首址
 *				 Color: 字符颜色   
 *				 mode: 0--文字背景色为白色   
 *						   1--文字悬浮 
 * 输出  ：无
 * 举例  ：PutChinese21(200,100,"好",0,0);
 * 注意	 ：如果输入大于1的汉字字符串，显示将会截断，只显示最前面一个汉字
 *********************************************************/    
void PutChinese(uint16_t Xpos,uint16_t Ypos,uint8_t *str,uint8_t mode) 
{
    uint8_t i,j;
    uint8_t buffer[32];
    uint16_t tmp_char=0;
   #ifdef SONG_TYPEFACE																	
    GetGBKCode(buffer,str); /* 取字模数据 */
   #endif
    for (i=0;i<16;i++)
    {
        tmp_char=buffer[i*2];
	      tmp_char=(tmp_char<<8);
        tmp_char|=buffer[2*i+1];
        for (j=0;j<16;j++)
        {
            
            if ( (tmp_char >> 15-j) & 0x01 == 0x01)
            {
                LCD_Draw_Point(Ypos+j,Xpos+i);
            }
            else
            {
                if ( mode == 0 )
                    LCD_Draw_ColorPoint(Ypos+j,Xpos+i,BACK_COLOR);//指定字体背景颜色
                else if ( mode == 1 )
                {
                    //不写入
                }	
                
                
            }
        }
    }
    
    
}

/******************************************************
 * 函数名：PutChinese_strings21
 * 描述  ：显示汉字字符串
 * 输入  : pos: 0~(319-16)
 *         Ypos: 0~(239-16)
 *				 str: 中文字符串首址
 *				 Color: 字符颜色   
 *				 mode: 0--文字背景色为白色   
 *						   1--文字悬浮 
 * 输出  ：无
 * 举例  ：PutChinese_strings2(200,100,"好人",0,0);
 * 注意	 ：无
 *********************************************************/     
void PutChinese_strings(uint16_t Xpos,uint16_t Ypos,uint8_t *str,uint8_t mode)	
{
    
    uint16_t Tmp_x, Tmp_y;
    uint8_t *tmp_str=str;
    Tmp_x = Ypos;
    Tmp_y = Xpos;
    
    while(*tmp_str != '\0')
    {
        PutChinese(Tmp_x,Tmp_y,tmp_str,mode);
        
        tmp_str += 2 ;
        Tmp_y += 16 ;	
    }       
}


void LCD_ShowMenuOSC(void)
{
	LCD_Clear(BLACK);//清屏为黑色
//	LCD_Draw_Circle(120, 120, 100);//画圆 半径r=100
//	LCD_Draw_Circle(120, 120, 80);//画圆 半径r=80
//	LCD_Draw_Circle(120, 120, 60);//画圆 半径r=60
//	LCD_Draw_Circle(120, 120, 40);//画圆 半径r=40
//	LCD_Draw_Circle(120, 120, 20);//画圆 半径r=20
//	LCD_Draw_Circle(120, 120, 1);//画圆 半径r=1



	LCD_ShowString(70, 10, 240, 16, 16, "Oscilloscope");//显示字符串，字体大小16*16
	LCD_DrawLine(1, 39, 239, 39);//第一行	
	LCD_DrawLine(1, 199, 239, 199);//最下一行		

	LCD_DrawLine(1, 39, 1, 199);//第一列
	LCD_DrawLine(239, 39, 239, 199);//第七列	

	for(uint8_t i=39;i<=199;i=i+3) {LCD_Draw_Point(40,i);}//第二列
	for(uint8_t i=39;i<=199;i=i+3) {LCD_Draw_Point(80,i);}//第三列
	for(uint8_t i=39;i<=199;i=i+3) {LCD_Draw_Point(120,i);}//第四列
	for(uint8_t i=39;i<=199;i=i+3) {LCD_Draw_Point(160,i);}//第五列		
	for(uint8_t i=39;i<=199;i=i+3) {LCD_Draw_Point(200,i);}//第六列			
	
	LCD_DrawLine(1, 79, 239, 79);//第二行	
	LCD_DrawLine(1, 119, 239, 119);//第三行	
	LCD_DrawLine(1, 159, 239, 159);//第四行	
		
	
	
	
	
	HAL_Delay(1500);//延时1秒

}
void LCD_ShowMenuSpectrum(void)
{
	LCD_Clear(BLACK);//清屏为蓝色	
	
	
	LCD_ShowString(75, 2, 240, 16, 16, "Spectrum");//显示字符串，字体大小16*16
	LCD_DrawLine(1, 19, 239, 19);//第一行	
	LCD_DrawLine(1, 219, 239, 219);//最下一行


	
	LCD_DrawLine(1, 19, 1, 219);//第一列
	LCD_DrawLine(239, 19, 239, 219);//第十二列	

	for(uint8_t i=19;i<=219;i=i+3) {LCD_Draw_Point(20,i);}//第二列
	for(uint8_t i=19;i<=219;i=i+3) {LCD_Draw_Point(40,i);}//第三列
	for(uint8_t i=19;i<=219;i=i+3) {LCD_Draw_Point(60,i);}//第四列
	for(uint8_t i=19;i<=219;i=i+3) {LCD_Draw_Point(80,i);}//第五列		
	for(uint8_t i=19;i<=219;i=i+3) {LCD_Draw_Point(100,i);}//第六列		
	for(uint8_t i=19;i<=219;i=i+3) {LCD_Draw_Point(120,i);}//第七列
	for(uint8_t i=19;i<=219;i=i+3) {LCD_Draw_Point(140,i);}//第八列
	for(uint8_t i=19;i<=219;i=i+3) {LCD_Draw_Point(160,i);}//第九列
	for(uint8_t i=19;i<=219;i=i+3) {LCD_Draw_Point(180,i);}//第十列		
	for(uint8_t i=19;i<=219;i=i+3) {LCD_Draw_Point(200,i);}//第十一列		
	for(uint8_t i=19;i<=219;i=i+3) {LCD_Draw_Point(220,i);}//第十一列			
	LCD_DrawLine(1, 39, 239, 39);//第二行	
	LCD_DrawLine(1, 59, 239, 59);//第三行	
	LCD_DrawLine(1, 79, 239, 79);//第四行	
	LCD_DrawLine(1, 99, 239, 99);//第五行	
	LCD_DrawLine(1, 119, 239, 119);//第六行
	LCD_DrawLine(1, 139, 239, 139);//第七行		
	LCD_DrawLine(1, 159, 239, 159);//第八行	
	LCD_DrawLine(1, 179, 239, 179);//第九行	
	LCD_DrawLine(1, 199, 239, 199);//第十行	
		
	
	
	
	
	HAL_Delay(1500);//延时1秒

}

void LCD_ShowMenuPWM(void)
{

	LCD_Clear(BLACK);//清屏为蓝色	
	
	
	LCD_ShowString(75, 2, 240, 16, 16, "PWM Signal");//显示字符串，字体大小16*16
	LCD_DrawRectangle(3, 19, 236, 239);
	LCD_DrawLine(3, 119, 236, 119);//第三行			
	
	
	LCD_ShowString(5, 2+20, 240, 16, 16, "3.3V");//显示字符串，字体大小16*16	
	LCD_ShowString(5+2, 2+80, 240, 16, 16, "0V");//显示字符串，字体大小16*16	

	for(uint8_t i=39;i<=79;i=i+3) {LCD_Draw_Point(i,22+8);}//3.3V行
	for(uint8_t i=39;i<=79;i=i+3) {LCD_Draw_Point(i,82+8);}//0V行
	
	LCD_DrawLine(54+40, 90, 59+40, 90);//第二行	
	LCD_DrawLine(59+40, 30, 119+40, 30);//第二行		
	LCD_DrawLine(119+40, 90, 179+40, 90);//第二行	
	LCD_DrawLine(179+40, 30, 184+40, 30);//第二行	
	for(uint8_t i=30;i<=90;i=i+1) {LCD_Draw_Point(99,i);}//第二列
	for(uint8_t i=30;i<=90;i=i+1) {LCD_Draw_Point(159,i);}//第二列
	for(uint8_t i=30;i<=90;i=i+1) {LCD_Draw_Point(219,i);}//第二列

	
	
	
	LCD_ShowString(5, 119+40, 240, 16, 16, "Frequency");//显示字符串，字体大小16*16	
	LCD_ShowString(5, 119+80, 240, 16, 16, "Duty Cycle");//显示字符串，字体大小16*16	
	
	HAL_Delay(1500);//延时1秒		
}


void LCD_ShowMenuDC(void)
{
	LCD_Clear(BLACK);//清屏为蓝色	
	
	
	LCD_ShowString(55, 2, 240, 16, 16, "DC Power Supply");//显示字符串，字体大小16*16
	LCD_DrawRectangle(3, 19, 236, 239);
	LCD_DrawLine(3, 79, 236, 79);//第三行			
	
	
	LCD_ShowString(10, 30, 240, 32, 32, "DC1");//显示字符串，字体大小16*16	
	LCD_ShowString(130, 30, 240, 32, 32, "DC2");//显示字符串，字体大小16*16	

	for(uint8_t i=19;i<=239;i=i+1) {LCD_Draw_Point(120,i);}//第四列
	
	
	
	LCD_ShowString(5, 119+40, 240, 16, 16, "ON");//显示字符串，字体大小16*16	
	LCD_ShowString(5, 119+80, 240, 16, 16, "OFF");//显示字符串，字体大小16*16	
	
	LCD_ShowString(125, 119+40, 240, 16, 16, "ON");//显示字符串，字体大小16*16	
	LCD_ShowString(125, 119+80, 240, 16, 16, "OFF");//显示字符串，字体大小16*16		
	
	HAL_Delay(1500);//延时1秒
}
void LCD_ShowMenuFG(void)
{
	LCD_Clear(BLACK);//清屏为蓝色	
	
	
	LCD_ShowString(55, 2, 240, 16, 16, "Function Generator");//显示字符串，字体大小16*16
	LCD_DrawRectangle(3, 19, 236, 239);
	LCD_DrawLine(3, 89, 236, 89);//第三行			
	
	LCD_DrawLine(3, 89+50, 236, 89+50);//第三行	
	LCD_DrawLine(3, 89+100, 236, 89+100);//第三行

	for(uint8_t i=19;i<=89;i=i+1) {LCD_Draw_Point(80,i);}//第二列
	for(uint8_t i=19;i<=89;i=i+1) {LCD_Draw_Point(160,i);}//第二列
	
	LCD_ShowString(5, 42, 240, 16, 16, "Triangle");//显示字符串，字体大小16*16		
	LCD_ShowString(85, 42, 240, 16, 16, "Rectangle");//显示字符串，字体大小16*16	
	LCD_ShowString(165, 42, 240, 16, 16, "Sine");//显示字符串，字体大小16*16		
	
	LCD_ShowString(5, 89+13, 240, 16, 16, "Amplitude");//显示字符串，字体大小16*16	
	LCD_ShowString(5, 89+13+50, 240, 16, 16, "Frequency");//显示字符串，字体大小16*16	
	LCD_ShowString(5, 89+13+100, 240, 16, 16, "DC Offset");//显示字符串，字体大小16*16		

	
	HAL_Delay(1500);//延时1秒

}


/**
 * @brief	写半个字的数据到LCD
 *
 * @param   cmd		需要发送的数据
 *
 * @return  void
 */
void LCD_Write_HalfWord(const uint16_t da)
{
    uint8_t data[2] = {0};

    data[0] = da >> 8;
    data[1] = da;

    LCD_DC(1);
    LCD_SPI_Send(data, 2);
}


/**
 * 设置数据写入LCD缓存区域
 *
 * @param   x1,y1	起点坐标
 * @param   x2,y2	终点坐标
 *
 * @return  void
 */
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_Write_Cmd(0x2a);
    LCD_Write_Data(x1 >> 8);
    LCD_Write_Data(x1);
    LCD_Write_Data(x2 >> 8);
    LCD_Write_Data(x2);

    LCD_Write_Cmd(0x2b);
    LCD_Write_Data(y1 >> 8);
    LCD_Write_Data(y1);
    LCD_Write_Data(y2 >> 8);
    LCD_Write_Data(y2);

    LCD_Write_Cmd(0x2C);
}

/**
 * 打开LCD显示
 *
 * @param   void
 *
 * @return  void
 */
void LCD_DisplayOn(void)
{

}
/**
 * 关闭LCD显示
 *
 * @param   void
 *
 * @return  void
 */
void LCD_DisplayOff(void)
{

}

/**
 * 以一种颜色清空LCD屏
 *
 * @param   color	清屏颜色
 *
 * @return  void
 */
void LCD_Clear(uint16_t color)
{
    uint16_t i, j;
    uint8_t data[2] = {0};

    data[0] = color >> 8;
    data[1] = color;

    LCD_Address_Set(0, 0, LCD_Width - 1, LCD_Height - 1);

    for(j = 0; j < LCD_Buf_Size / 2; j++)
    {
        lcd_buf[j * 2] =  data[0];
        lcd_buf[j * 2 + 1] =  data[1];
    }

    LCD_DC(1);

    for(i = 0; i < (LCD_TOTAL_BUF_SIZE / LCD_Buf_Size); i++)
    {
        LCD_SPI_Send(lcd_buf, LCD_Buf_Size);
    }
}

/**
 * 用一个颜色填充整个区域
 *
 * @param   x_start,y_start     起点坐标
 * @param   x_end,y_end			终点坐标
 * @param   color       		填充颜色
 *
 * @return  void
 */
void LCD_Fill(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color)
{
    uint16_t i = 0;
    uint32_t size = 0, size_remain = 0;

    size = (x_end - x_start + 1) * (y_end - y_start + 1) * 2;

    if(size > LCD_Buf_Size)
    {
        size_remain = size - LCD_Buf_Size;
        size = LCD_Buf_Size;
    }

    LCD_Address_Set(x_start, y_start, x_end, y_end);

    while(1)
    {
        for(i = 0; i < size / 2; i++)
        {
            lcd_buf[2 * i] = color >> 8;
            lcd_buf[2 * i + 1] = color;
        }

        LCD_DC(1);
        LCD_SPI_Send(lcd_buf, size);

        if(size_remain == 0)
            break;

        if(size_remain > LCD_Buf_Size)
        {
            size_remain = size_remain - LCD_Buf_Size;
        }

        else
        {
            size = size_remain;
            size_remain = 0;
        }
    }
}

/**
 * 画点函数
 *
 * @param   x,y		画点坐标
 *
 * @return  void
 */
void LCD_Draw_Point(uint16_t x, uint16_t y)
{
    LCD_Address_Set(x, y, x, y);
    LCD_Write_HalfWord(POINT_COLOR);
}

/**
 * 画点带颜色函数
 *
 * @param   x,y		画点坐标
 *
 * @return  void
 */
void LCD_Draw_ColorPoint(uint16_t x, uint16_t y,uint16_t color)
{
    LCD_Address_Set(x, y, x, y);
    LCD_Write_HalfWord(color);
}



/**
 * @brief	画线函数(直线、斜线)
 *
 * @param   x1,y1	起点坐标
 * @param   x2,y2	终点坐标
 *
 * @return  void
 */
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, row, col;
    uint32_t i = 0;

    if(y1 == y2)
    {
        /*快速画水平线*/
        LCD_Address_Set(x1, y1, x2, y2);

        for(i = 0; i < x2 - x1; i++)
        {
            lcd_buf[2 * i] = POINT_COLOR >> 8;
            lcd_buf[2 * i + 1] = POINT_COLOR;
        }

        LCD_DC(1);
        LCD_SPI_Send(lcd_buf, (x2 - x1) * 2);
        return;
    }

    delta_x = x2 - x1;
    delta_y = y2 - y1;
    row = x1;
    col = y1;

    if(delta_x > 0)incx = 1;

    else if(delta_x == 0)incx = 0;

    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if(delta_y > 0)incy = 1;

    else if(delta_y == 0)incy = 0;

    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if(delta_x > delta_y)distance = delta_x;

    else distance = delta_y;

    for(t = 0; t <= distance + 1; t++)
    {
        LCD_Draw_Point(row, col);
        xerr += delta_x ;
        yerr += delta_y ;

        if(xerr > distance)
        {
            xerr -= distance;
            row += incx;
        }

        if(yerr > distance)
        {
            yerr -= distance;
            col += incy;
        }
    }
}

/**
 * @brief	画一个矩形
 *
 * @param   x1,y1	起点坐标
 * @param   x2,y2	终点坐标
 *
 * @return  void
 */
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_DrawLine(x1, y1, x2, y1);
    LCD_DrawLine(x1, y1, x1, y2);
    LCD_DrawLine(x1, y2, x2, y2);
    LCD_DrawLine(x2, y1, x2, y2);
}

/**
 * @brief	画一个圆
 *
 * @param   x0,y0	圆心坐标
 * @param   r       圆半径
 *
 * @return  void
 */
void LCD_Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r)
{
    int a, b;
    int di;
    a = 0;
    b = r;
    di = 3 - (r << 1);

    while(a <= b)
    {
        LCD_Draw_Point(x0 - b, y0 - a);
        LCD_Draw_Point(x0 + b, y0 - a);
        LCD_Draw_Point(x0 - a, y0 + b);
        LCD_Draw_Point(x0 - b, y0 - a);
        LCD_Draw_Point(x0 - a, y0 - b);
        LCD_Draw_Point(x0 + b, y0 + a);
        LCD_Draw_Point(x0 + a, y0 - b);
        LCD_Draw_Point(x0 + a, y0 + b);
        LCD_Draw_Point(x0 - b, y0 + a);
        a++;

        //Bresenham
        if(di < 0)di += 4 * a + 6;

        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }

        LCD_Draw_Point(x0 + a, y0 + b);
    }
}

/**
 * @brief	显示一个ASCII码字符
 *
 * @param   x,y		显示起始坐标
 * @param   chr		需要显示的字符
 * @param   size	字体大小(支持16/24/32号字体)
 *
 * @return  void
 */
void LCD_ShowChar(uint16_t x, uint16_t y, char chr, uint8_t size)
{
    uint8_t temp, t1, t;
    uint8_t csize;		//得到字体一个字符对应点阵集所占的字节数
    uint16_t colortemp;
    uint8_t sta;

    chr = chr - ' '; //得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库）

    if((x > (LCD_Width - size / 2)) || (y > (LCD_Height - size)))	return;

    LCD_Address_Set(x, y, x + size / 2 - 1, y + size - 1);//(x,y,x+8-1,y+16-1)

    if((size == 16) || (size == 32) )	//16和32号字体
    {
        csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);

        for(t = 0; t < csize; t++)
        {
            if(size == 16)temp = asc2_1608[chr][t];	//调用1608字体

            else if(size == 32)temp = asc2_3216[chr][t];	//调用3216字体

            else return;			//没有的字库

            for(t1 = 0; t1 < 8; t1++)
            {
                if(temp & 0x80) colortemp = POINT_COLOR;

                else colortemp = BACK_COLOR;

                LCD_Write_HalfWord(colortemp);
                temp <<= 1;
            }
        }
    }

	else if  (size == 12)	//12号字体
	{
        csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);

        for(t = 0; t < csize; t++)
        {
            temp = asc2_1206[chr][t];

            for(t1 = 0; t1 < 6; t1++)
            {
                if(temp & 0x80) colortemp = POINT_COLOR;

                else colortemp = BACK_COLOR;

                LCD_Write_HalfWord(colortemp);
                temp <<= 1;
            }
        }
    }
	
    else if(size == 24)		//24号字体
    {
        csize = (size * 16) / 8;

        for(t = 0; t < csize; t++)
        {
            temp = asc2_2412[chr][t];

            if(t % 2 == 0)sta = 8;

            else sta = 4;

            for(t1 = 0; t1 < sta; t1++)
            {
                if(temp & 0x80) colortemp = POINT_COLOR;

                else colortemp = BACK_COLOR;

                LCD_Write_HalfWord(colortemp);
                temp <<= 1;
            }
        }
    }
}

/**
 * @brief	m^n函数
 *
 * @param   m,n		输入参数
 *
 * @return  m^n次方
 */
static uint32_t LCD_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while(n--)result *= m;

    return result;
}

/**
 * @brief	显示数字,高位为0不显示
 *
 * @param   x,y		起点坐标
 * @param   num		需要显示的数字,数字范围(0~4294967295)
 * @param   len		需要显示的位数
 * @param   size	字体大小
 *
 * @return  void
 */
void LCD_ShowNum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for(t = 0; t < len; t++)
    {
        temp = (num / LCD_Pow(10, len - t - 1)) % 10;

        if(enshow == 0 && t < (len - 1))
        {
            if(temp == 0)
            {
                LCD_ShowChar(x + (size / 2)*t, y, ' ', size);
                continue;
            }

            else enshow = 1;
        }

        LCD_ShowChar(x + (size / 2)*t, y, temp + '0', size);
    }
}



/**
 * @brief	显示数字,高位为0,可以控制显示为0还是不显示
 *
 * @param   x,y		起点坐标
 * @param   num		需要显示的数字,数字范围(0~999999999)
 * @param   len		需要显示的位数
 * @param   size	字体大小
 * @param   mode	1:高位显示0		0:高位不显示
 *
 * @return  void
 */
void LCD_ShowxNum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for(t = 0; t < len; t++)
    {
        temp = (num / LCD_Pow(10, len - t - 1)) % 10;

        if(enshow == 0 && t < (len - 1))
        {
            if(temp == 0)
            {
                if(mode)LCD_ShowChar(x + (size / 2)*t, y, '0', size);

                else
                    LCD_ShowChar(x + (size / 2)*t, y, ' ', size);

                continue;
            }

            else enshow = 1;
        }

        LCD_ShowChar(x + (size / 2)*t, y, temp + '0', size);
    }
}


/**
 * @brief	显示字符串
 *
 * @param   x,y		起点坐标
 * @param   width	字符显示区域宽度
 * @param   height	字符显示区域高度
 * @param   size	字体大小
 * @param   p		字符串起始地址
 *
 * @return  void
 */
void LCD_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p)
{
    uint8_t x0 = x;
    width += x;
    height += y;

    while((*p <= '~') && (*p >= ' ')) //判断是不是非法字符!
    {
        if(x >= width)
        {
            x = x0;
            y += size;
        }

        if(y >= height)break; //退出

        LCD_ShowChar(x, y, *p, size);
        x += size / 2;
        p++;
    }
}


/**
 * @brief	显示图片
 *
 * @remark	Image2Lcd取模方式：	C语言数据/水平扫描/16位真彩色(RGB565)/高位在前		其他的不要选
 *
 * @param   x,y		起点坐标
 * @param   width	图片宽度
 * @param   height	图片高度
 * @param   p		图片缓存数据起始地址
 *
 * @return  void
 */
void LCD_Show_Image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *p)
{
    if(x + width > LCD_Width || y + height > LCD_Height)
    {
        return;
    }

    LCD_Address_Set(x, y, x + width - 1, y + height - 1);

    LCD_DC(1);

	if(width * height*2>65535)
	{
		LCD_SPI_Send((uint8_t *)p, 65535);
		LCD_SPI_Send((uint8_t *)(p+65535), width*height*2-65535);
	}
	else
	{
		LCD_SPI_Send((uint8_t *)p, width * height * 2);
	}
}

