#include "oled.hpp"

namespace OLED_n
{
    BSP_DWT_n::BSP_DWT_c* dwt_time_oled = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();


    /**
     * @brief 硬件OLED初始化构造函数
     * 
     * @param BSP_I2C_n::HW_I2C_Config_s HW_I2C_Config 
     */
    OLED_c::OLED_c(BSP_I2C_n::HW_I2C_Config_s HW_I2C_Config)
                        ://设置oled参数
                        BSP_I2C_c(HW_I2C_Config)                   
    {
        oled_mode_ = Hardware;
    }

    /**
     * @brief 模拟OLED初始化构造函数
     * 
     * @param BSP_I2C_n::SW_I2C_Config_s SW_I2C_Config 
     */
    OLED_c::OLED_c(BSP_I2C_n::SW_I2C_Config_s SW_I2C_Config)
                        ://设置oled参数
                        BSP_I2C_c(SW_I2C_Config) 
    {
        oled_mode_ = Software;
    }

/******************************硬件oled库*********************************************** */
    /**
     * @brief          初始化OLED模块，
     * @param[in]      none
     * @retval         none
     */
    uint8_t OLED_Init_CMD[ ] =
    {
        0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6, 0xA8,
        0x3F, 0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD8, 0x05, 0xD9, 0xF1,
        0xDA, 0x12, 0xDB, 0x30, 0x8D, 0x14, 0xAF, 0x20, 0x00
    };

    /**
     * @brief OLED初始化函数
     * 
     */
    void OLED_c::HW_Init(void)
    {
        HW_I2CAccessMem(OLED_CMD, OLED_Init_CMD, 29, BSP_I2C_n::I2C_WRITE_MEM, I2C_MEMADD_SIZE_8BIT);
    }

    /**
     * @brief          操作GRAM内存(128*8char数组)
     * @param[in]      pen: 操作类型.
                        PEN_CLEAR: 设置为0x00
                        PEN_WRITE: 设置为0xff
    * @retval         none
    */
    void OLED_c::HW_Operate_Gram(pen_typedef pen)
    {
        if (pen == PEN_WRITE)
        {
                memset(OLED_GRAM,0xff,sizeof(OLED_GRAM));
        }
        else if(pen == PEN_CLEAR)
        {
                memset(OLED_GRAM,0x00,sizeof(OLED_GRAM));
        }    
    }

    /**
     * @brief          设置光标起点(x,y)
     * @param[in]      x:x轴, 从 0 到 127
     * @param[in]      y:y轴, 从 0 到 7
     * @retval         none
     */
    void OLED_c::HW_Set_Pos(uint8_t x, uint8_t y)
    {
        OLED_CMDbuf[y][0] = 0x00;
        OLED_CMDbuf[y][1] = 0xb0 + y;
        OLED_CMDbuf[y][2] = 0x10;
        OLED_CMDbuf[y][3] = 0x00;
    }

    /**
     * @brief          操作GRAM中的一个位，相当于操作屏幕的一个点
     * @param[in]      x:x轴,  [0,X_WIDTH-1]
     * @param[in]      y:y轴,  [0,Y_WIDTH-1]
     * @param[in]      pen: 操作类型,
                            PEN_CLEAR: 设置 (x,y) 点为 0
                            PEN_WRITE: 设置 (x,y) 点为 1
                            PEN_INVERSION: (x,y) 值反转
    * @retval         none
    */
    void OLED_c::HW_Draw_Point(int8_t x, int8_t y, pen_typedef pen)
    {
        uint8_t page = 0, row = 0;

        /* check the corrdinate */
        if ((x < 0) || (x > (X_WIDTH - 1)) || (y < 0) || (y > (Y_WIDTH - 1)))
        {
            return;
        }
        page = y / 8;
        row = y % 8;

        if (pen == PEN_WRITE)
        {
            OLED_GRAM[x][page] |= 1 << row;
        }
        else if (pen == PEN_INVERSION)
        {
            OLED_GRAM[x][page] ^= 1 << row;
        }
        else
        {
            OLED_GRAM[x][page] &= ~(1 << row);
        }
    }

    /**
     * @brief          画一条直线，从(x1,y1)到(x2,y2)
     * @param[in]      x1: 起点
     * @param[in]      y1: 起点
     * @param[in]      x2: 终点
     * @param[in]      y2: 终点
     * @param[in]      pen: 操作类型,PEN_CLEAR,PEN_WRITE,PEN_INVERSION.
     * @retval         none
     */
    
    void OLED_c::HW_Draw_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen)
    {
        uint8_t col = 0, row = 0;
        uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
        float k = 0.0f, b = 0.0f;

        if (y1 == y2)
        {
            (x1 <= x2) ? (x_st = x1):(x_st = x2);
            (x1 <= x2) ? (x_ed = x2):(x_ed = x1);

            for (col = x_st; col <= x_ed; col++)
            {
                HW_Draw_Point(col, y1, pen);
            }
        }
        else if (x1 == x2)
        {
            (y1 <= y2) ? (y_st = y1):(y_st = y2);
            (y1 <= y2) ? (y_ed = y2):(y_ed = y1);

            for (row = y_st; row <= y_ed; row++)
            {
                HW_Draw_Point(x1, row, pen);
            }
        }
        else
        {
            k = ((float)(y2 - y1)) / (x2 - x1);
            b = (float)y1 - k * x1;

            (x1 <= x2) ? (x_st = x1):(x_st = x2);
            (x1 <= x2) ? (x_ed = x2):(x_ed = x2);

            for (col = x_st; col <= x_ed; col++)
            {
                HW_Draw_Point(col, (uint8_t)(col * k + b), pen);
            }
        }
    }

    /**
     * @brief          显示一个字符
     * @param[in]      row: 字符的开始行
     * @param[in]      col: 字符的开始列
     * @param[in]      chr: 字符
     * @retval         none
     */
    void OLED_c::HW_Show_Char(uint8_t row, uint8_t col, uint8_t chr)
    {
        uint8_t x = col * 6;
        uint8_t y = row * 12;
        uint8_t temp, t, t1;
        uint8_t y0 = y;
        chr = chr - ' ';

        for (t = 0; t < 12; t++)
        {
            temp = asc2_1206[chr][t];

            for (t1 = 0; t1 < 8; t1++)
            {
                if (temp&0x80)
                    HW_Draw_Point(x, y, PEN_WRITE);
                else
                    HW_Draw_Point(x, y, PEN_CLEAR);

                temp <<= 1;
                y++;
                if ((y - y0) == 12)
                {
                    y = y0;
                    x++;
                    break;
                }
            }
        }
    }

    /**
     * @brief          显示一个字符串
     * @param[in]      row: 字符串的开始行
     * @param[in]      col: 字符串的开始列
     * @param[in]      chr: 字符串
     * @retval         none
     */
    void OLED_c::HW_Show_String(uint8_t row, uint8_t col, uint8_t *chr)
    {
        uint8_t n =0;

        while (chr[n] != '\0')
        {
            HW_Show_Char(row, col, chr[n]);
            col++;

            if (col > 20)
            {
                col = 0;
                row += 1;
            }
            n++;
        }
    }

    /**
     * @brief          发送数据到OLED的GRAM
     * @param[in]      none
     * @retval         none
     */
    void OLED_c::HW_Refresh_Gram(void)
    {	
        uint8_t i;
        uint16_t j;
            
        if(BufFinshFlag == 0)
        {
            for(i = 0; i < 8; i ++ )
            {
                HW_Set_Pos(0, i);
                for(j = 0;j < 128; j ++)
                {
                        OLED_GRAMbuf[i][j] = OLED_GRAM[j][i];  //OLED_GRAM[128][8]
                }
            }
            BufFinshFlag = 1;
            HW_I2C_Transmit(OLED_CMDbuf[0],4);
        }
    }

    uint32_t OLED_c::HW_Pow(uint8_t m, uint8_t n)
    {
    uint32_t result = 1;

    while (n--)
        result *= m;

    return result;
    }

    /**
     * @brief  OLED显示数字（十进制，正数）
     * @param  Line 起始行位置，范围：1~4
     * @param  Column 起始列位置，范围：1~16
     * @param  Number 要显示的数字，范围：0~4294967295
     * @param  Length 要显示数字的长度，范围：1~10
     * @retval 无
     */
    void OLED_c::HW_Show_Num(uint8_t x, uint8_t y, uint32_t num, uint8_t len)
    {
        uint8_t t, temp;
        uint8_t enshow = 0;

        for (t = 0; t < len; t++)
        {
            temp = (num / HW_Pow(10, len - t - 1)) % 10;

            if (enshow == 0 && t < (len - 1))
            {
            if (temp == 0)
            {
                HW_Show_Char(x, y + t, ' ');
                continue;
            }
            else
                enshow = 1;
            }

            HW_Show_Char(x, y + t, temp + '0');
        }
    }


    /*****************************模拟oled库********************************************** */
    /*引脚初始化*/
    void OLED_c::SW_I2C_Init(void)
    {
        
        SW_I2C_W_SCL(1);
        SW_I2C_W_SDA(1);
    }
    
    /**
     * @brief  I2C开始
     * @param  无
     * @retval 无
     */
    void OLED_c::SW_I2C_Start(void)
    {
        SW_I2C_W_SDA(1);
        dwt_time_oled->ECF_DWT_Delay_ms(I2C_DELAY_TIME);
        SW_I2C_W_SCL(1);
        dwt_time_oled->ECF_DWT_Delay_ms(I2C_DELAY_TIME);
        SW_I2C_W_SDA(0);
        dwt_time_oled->ECF_DWT_Delay_ms(I2C_DELAY_TIME);
        SW_I2C_W_SCL(0);
    }
    
    /**
     * @brief  I2C停止
     * @param  无
     * @retval 无
     */
    void OLED_c::SW_I2C_Stop(void)
    {
        SW_I2C_W_SDA(0);
        dwt_time_oled->ECF_DWT_Delay_ms(I2C_DELAY_TIME);
        SW_I2C_W_SCL(1);
        dwt_time_oled->ECF_DWT_Delay_ms(I2C_DELAY_TIME);
        SW_I2C_W_SDA(1);
    }
    
    /**
     * @brief  I2C发送一个字节
     * @param  Byte 要发送的一个字节
     * @retval 无
     */
    void OLED_c::SW_I2C_SendByte(uint8_t Byte)
    {
        uint8_t i;
        for (i = 0; i < 8; i++)
        {
            SW_I2C_W_SDA(Byte & (0x80 >> i));
            dwt_time_oled->ECF_DWT_Delay_ms(I2C_DELAY_TIME);
            SW_I2C_W_SCL(1);
            dwt_time_oled->ECF_DWT_Delay_ms(I2C_DELAY_TIME);
            SW_I2C_W_SCL(0);
        }
        dwt_time_oled->ECF_DWT_Delay_ms(I2C_DELAY_TIME);
        SW_I2C_W_SCL(1);	//额外的一个时钟，不处理应答信号
        dwt_time_oled->ECF_DWT_Delay_ms(I2C_DELAY_TIME);
        SW_I2C_W_SCL(0);
        dwt_time_oled->ECF_DWT_Delay_ms(I2C_DELAY_TIME);
    }
    
    /**
     * @brief  OLED写命令
     * @param  Command 要写入的命令
     * @retval 无
     */
    void OLED_c::SW_WriteCommand(uint8_t Command)
    {
        SW_I2C_Start();
        SW_I2C_SendByte(0x78);		//从机地址
        SW_I2C_SendByte(0x00);		//写命令
        SW_I2C_SendByte(Command); 
        SW_I2C_Stop();
    }
    
    /**
     * @brief  OLED写数据
     * @param  Data 要写入的数据
     * @retval 无
     */
    void OLED_c::SW_WriteData(uint8_t Data)
    {
        SW_I2C_Start();
        SW_I2C_SendByte(0x78);		//从机地址
        SW_I2C_SendByte(0x40);		//写数据
        SW_I2C_SendByte(Data);
        SW_I2C_Stop();
    }
    
    /**
     * @brief  OLED设置光标位置
     * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
     * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
     * @retval 无
     */
    void OLED_c::SW_SetCursor(uint8_t Y, uint8_t X)
    {
        SW_WriteCommand(0xB0 | Y);					//设置Y位置
        SW_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置X位置低4位
        SW_WriteCommand(0x00 | (X & 0x0F));			//设置X位置高4位
    }

    /**
     * @brief  OLED清屏
     * @param  无
     * @retval 无
     */
    void OLED_c::SW_Clear(void)
    {  
        uint8_t i, j;
        for (j = 0; j < 8; j++)
        {
            SW_SetCursor(j, 0);
            for(i = 0; i < 128; i++)
            {
                SW_WriteData(0x00);
            }
        }
    }

    /**
     * @brief  OLED显示一个字符
     * @param  Line 行位置，范围：1~4
     * @param  Column 列位置，范围：1~16
     * @param  Char 要显示的一个字符，范围：ASCII可见字符
     * @retval 无
     */
    void OLED_c::SW_Show_Char(uint8_t Line, uint8_t Column, char Char)
    {      	
        uint8_t i;
        SW_SetCursor((Line - 1) * 2, (Column - 1) * 8);		//设置光标位置在上半部分
        for (i = 0; i < 8; i++)
        {
            SW_WriteData(OLED_F8x16[Char - ' '][i]);			//显示上半部分内容
        }
        SW_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	//设置光标位置在下半部分
        for (i = 0; i < 8; i++)
        {
            SW_WriteData(OLED_F8x16[Char - ' '][i + 8]);		//显示下半部分内容
        }
    }
    
    /**
     * @brief  OLED显示字符串
     * @param  Line 起始行位置，范围：1~4
     * @param  Column 起始列位置，范围：1~16
     * @param  String 要显示的字符串，范围：ASCII可见字符
     * @retval 无
     */
    void OLED_c::SW_Show_String(uint8_t Line, uint8_t Column, uint8_t *String)
    {
        uint8_t i;
        for (i = 0; String[i] != '\0'; i++)
        {
            SW_Show_Char(Line, Column + i, String[i]);
        }
    }

    /**
     * @brief  OLED次方函数
     * @retval 返回值等于X的Y次方
     */
    uint32_t OLED_c::SW_Pow(uint32_t X, uint32_t Y)
    {
        uint32_t Result = 1;
        while (Y--)
        {
            Result *= X;
        }
        return Result;
    }

    /**
     * @brief  OLED显示数字（十进制，正数）
     * @param  Line 起始行位置，范围：1~4
     * @param  Column 起始列位置，范围：1~16
     * @param  Number 要显示的数字，范围：0~4294967295
     * @param  Length 要显示数字的长度，范围：1~10
     * @retval 无
     */
    void OLED_c::SW_Show_Num(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
    {
        uint8_t i;
        for (i = 0; i < Length; i++)							
        {
            SW_Show_Char(Line, Column + i, Number / SW_Pow(10, Length - i - 1) % 10 + '0');
        }
    }

    /**
     * @brief  模拟OLED初始化
     * @param  无
     * @retval 无
     */
    void OLED_c::SW_Init(void)
    {
        uint32_t i, j;
        
        for (i = 0; i < 1000; i++)			//上电延时
        {
            for (j = 0; j < 1000; j++);
        }
        
        SW_I2C_Init();			//端口初始化
        
        SW_WriteCommand(0xAE);	//关闭显示
        
        SW_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
        SW_WriteCommand(0x80);
        
        SW_WriteCommand(0xA8);	//设置多路复用率
        SW_WriteCommand(0x3F);
        
        SW_WriteCommand(0xD3);	//设置显示偏移
        SW_WriteCommand(0x00);
        
        SW_WriteCommand(0x40);	//设置显示开始行
        
        SW_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
        
        SW_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置
    
        SW_WriteCommand(0xDA);	//设置COM引脚硬件配置
        SW_WriteCommand(0x12);
        
        SW_WriteCommand(0x81);	//设置对比度控制
        SW_WriteCommand(0xCF);
    
        SW_WriteCommand(0xD9);	//设置预充电周期
        SW_WriteCommand(0xF1);
    
        SW_WriteCommand(0xDB);	//设置VCOMH取消选择级别
        SW_WriteCommand(0x30);
    
        SW_WriteCommand(0xA4);	//设置整个显示打开/关闭
    
        SW_WriteCommand(0xA6);	//设置正常/倒转显示
    
        SW_WriteCommand(0x8D);	//设置充电泵
        SW_WriteCommand(0x14);
    
        SW_WriteCommand(0xAF);	//开启显示
            
        SW_Clear();				//OLED清屏
    }


/************************OLED主函数*************************************************** */
    /**
     * @brief OLED初始化函数
     */
    void OLED_c::OLED_Init(void)
    {
        switch(oled_mode_)
        {
        case Hardware:
            HW_Init();
            break;
        case Software:
            SW_Init();
            break;
        default:
            while (1)
                ; // 未知传输模式, 程序停止 
            break;
        }
    }

    /**
    * @brief  OLED清屏
    * @param  无
    * @retval 无
    */  
    void OLED_c::OLED_Clear(void)
    {
        switch(oled_mode_)
        {
        case Hardware:
            HW_Operate_Gram(PEN_CLEAR);
            break;
        case Software:
            SW_Clear();
            break;
        default:
            while (1)
                ; // 未知传输模式, 程序停止 
            break;
        }
    }

    /**
     * @brief          显示一个字符
     * @param[in]      row: 字符的开始行
     * @param[in]      col: 字符的开始列
     * @param[in]      chr: 字符
     * @retval         none
     */
    void OLED_c::OLED_Show_Char(uint8_t Row, uint8_t Column, uint8_t chr)
    {
        switch(oled_mode_)
        {
        case Hardware:
            HW_Show_Char(Row, Column, chr);
            break;
        case Software:
            SW_Show_Char(Row, Column, chr);
            break;
        default:
            while (1)
                ; // 未知传输模式, 程序停止 
            break;
        }
    }

    /**
     * @brief          显示一个字符串
     * @param[in]      row: 字符串的开始行
     * @param[in]      col: 字符串的开始列
     * @param[in]      chr: 字符串
     * @retval         none
     */
    void OLED_c::OLED_Show_String(uint8_t Row, uint8_t Column, uint8_t *chr)
    {
        switch(oled_mode_)
        {
        case Hardware:
            HW_Show_String(Row, Column, chr);
            break;
        case Software:
            SW_Show_String(Row, Column, chr);
            break;
        default:
            while (1)
                ; // 未知传输模式, 程序停止 
            break;
        }
    }

    /**
     * @brief  OLED显示数字（十进制，正数）
     * @param  Line 起始行位置，范围：1~4
     * @param  Column 起始列位置，范围：1~16
     * @param  Number 要显示的数字，范围：0~4294967295
     * @param  Length 要显示数字的长度，范围：1~10
     * @retval 无
     */
    void OLED_c::OLED_Show_Num(uint8_t Row, uint8_t Column, uint32_t Number, uint8_t Length)
    {
        switch(oled_mode_)
        {
        case Hardware:
            HW_Show_Num(Row, Column, Number, Length);
            break;
        case Software:
            SW_Show_Num(Row, Column, Number, Length);
            break;
        default:
            while (1)
                ; // 未知传输模式, 程序停止 
            break;
        }
    }
}

 