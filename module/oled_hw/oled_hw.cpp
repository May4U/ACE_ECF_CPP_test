#include "oled.hpp"

uint8_t OLED_GRAM[128][8];
uint8_t OLED_GRAMbuf[8][128];
uint8_t OLED_CMDbuf[8][4] = {0};
uint8_t I2C1_MemTxFinshFlag = 1;
uint8_t CountFlag = 0; 
uint8_t BufFinshFlag = 0; 

HW_I2C_Config_s oled_init_config = {&hi2c1, 0x78, I2C_DMA_MODE};

void oled_callback(Bsp_I2C_c *register_instance)
{
    switch(register_instance->Callback_type_)
    {
    case I2C_Master:
        if(CountFlag == 7)
        {
          BufFinshFlag = 0;
          CountFlag = 0;
        }
        if(BufFinshFlag)
        {
          CountFlag ++;
          register_instance->HW_I2C_Transmit(OLED_CMDbuf[CountFlag],4);
        }
        break;
    case I2C_Mem:
        if(BufFinshFlag)
        {
          register_instance->HW_I2CAccessMem(0x40,OLED_GRAMbuf[CountFlag],128,I2C_WRITE_MEM, I2C_MEMADD_SIZE_8BIT);
        }
        break;
    default:
			while (1)
				; // 未知传输模式, 程序停止 
			break;
    }
}

Bsp_I2C_c oled_module(&oled_init_config, oled_callback);


uint32_t oled_pow(uint8_t m, uint8_t n)
{
  uint32_t result = 1;

  while (n--)
    result *= m;

  return result;
}

uint8_t check_num_len(uint32_t num)
{
  uint32_t tmp;
	uint8_t i;
  for(i = 1; i < 10; i++)
  {
    tmp = oled_pow(10, i);
    if(num < tmp)
    {
      return i;
    }
  }
	return 0;
}

void OLED_show_num(uint8_t x, uint8_t y, uint32_t num, uint8_t mode, uint8_t len)
{
  uint8_t t, temp;
  uint8_t enshow = 0;

  for (t = 0; t < len; t++)
  {
    temp = (num / oled_pow(10, len - t - 1)) % 10;

    if (enshow == 0 && t < (len - 1))
    {
      if (temp == 0)
      {
        if (mode == 0)
          OLED_show_char(x, y + t, ' ');
        else
          OLED_show_char(x, y + t, '0');
        continue;
      }
      else
        enshow = 1;
    }

    OLED_show_char(x, y + t, temp + '0');
  }
}

void OLED_show_floatnum(uint8_t x, uint8_t y, float num, uint8_t mode)
{
  int32_t m, n;
  float R;
  uint8_t chartemp[6], i;
	
  R = num;
  m = R / 1;
  n = (R - m) * 1000;
  if(n < 0) n = -n;

  i = check_num_len(m);

  OLED_show_num(x, y, m, mode, i);

  chartemp[0] = '.';
  chartemp[1] = n / 100 + 48;
  chartemp[2] = n / 10 % 10 + 48;
  chartemp[3] = n % 10 + 48;
  chartemp[4] = ' ';
  chartemp[5] = '\0';
  OLED_show_string(x, y + i, chartemp);
}

/**
  * @brief          initialize the oled device
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ???OLED??,
  * @param[in]      none
  * @retval         none
  */
uint8_t OLED_Init_CMD[ ] =
{
	0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6, 0xA8,
	0x3F, 0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD8, 0x05, 0xD9, 0xF1,
	0xDA, 0x12, 0xDB, 0x30, 0x8D, 0x14, 0xAF, 0x20, 0x00
};

void OLED_init(void)
{
	oled_module.HW_I2CAccessMem(0x00, OLED_Init_CMD, 29, I2C_WRITE_MEM, I2C_MEMADD_SIZE_8BIT);
  
}

/**
  * @brief          operate the graphic ram(size: 128*8 char)
  * @param[in]      pen: the type of operate.
                    PEN_CLEAR: set ram to 0x00
                    PEN_WRITE: set ram to 0xff
                    PEN_INVERSION: bit inversion 
  * @retval         none
  */
/**
  * @brief          ??GRAM??(128*8char??)
  * @param[in]      pen: ????.
                    PEN_CLEAR: ???0x00
                    PEN_WRITE: ???0xff
  * @retval         none
  */
void OLED_operate_gram(pen_typedef pen)
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
  * @brief          cursor set to (x,y) point
  * @param[in]      x:X-axis, from 0 to 127
  * @param[in]      y:Y-axis, from 0 to 7
  * @retval         none
  */
/**
  * @brief          ??????(x,y)
  * @param[in]      x:x?, ? 0 ? 127
  * @param[in]      y:y?, ? 0 ? 7
  * @retval         none
  */
void OLED_set_pos(uint8_t x, uint8_t y)
{
	OLED_CMDbuf[y][0] = 0x00;
	OLED_CMDbuf[y][1] = 0xb0 + y;
	OLED_CMDbuf[y][2] = 0x10;
	OLED_CMDbuf[y][3] = 0x00;
	
}


/**
  * @brief          draw one bit of graphic raw, operate one point of screan(128*64)
  * @param[in]      x: x-axis, [0, X_WIDTH-1]
  * @param[in]      y: y-axis, [0, Y_WIDTH-1]
  * @param[in]      pen: type of operation,
                        PEN_CLEAR: set (x,y) to 0
                        PEN_WRITE: set (x,y) to 1
                        PEN_INVERSION: (x,y) value inversion 
  * @retval         none
  */
/**
  * @brief          ??GRAM?????,???????????
  * @param[in]      x:x?,  [0,X_WIDTH-1]
  * @param[in]      y:y?,  [0,Y_WIDTH-1]
  * @param[in]      pen: ????,
                        PEN_CLEAR: ?? (x,y) ?? 0
                        PEN_WRITE: ?? (x,y) ?? 1
                        PEN_INVERSION: (x,y) ???
  * @retval         none
  */
void OLED_draw_point(int8_t x, int8_t y, pen_typedef pen)
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
  * @brief          draw a line from (x1, y1) to (x2, y2)
  * @param[in]      x1: the start point of line
  * @param[in]      y1: the start point of line
  * @param[in]      x2: the end point of line
  * @param[in]      y2: the end point of line
  * @param[in]      pen: type of operation,PEN_CLEAR,PEN_WRITE,PEN_INVERSION.
  * @retval         none
  */
/**
  * @brief          画一条直线，从(x1,y1)到(x2,y2)
  * @param[in]      x1: 起点
  * @param[in]      y1: 起点
  * @param[in]      x2: 终点
  * @param[in]      y2: 终点
  * @param[in]      pen: 操作类型,PEN_CLEAR,PEN_WRITE,PEN_INVERSION.
  * @retval         none
  */
  
void OLED_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen)
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
            OLED_draw_point(col, y1, pen);
        }
    }
    else if (x1 == x2)
    {
        (y1 <= y2) ? (y_st = y1):(y_st = y2);
        (y1 <= y2) ? (y_ed = y2):(y_ed = y1);

        for (row = y_st; row <= y_ed; row++)
        {
            OLED_draw_point(x1, row, pen);
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
            OLED_draw_point(col, (uint8_t)(col * k + b), pen);
        }
    }
}


/**
  * @brief          show a character
  * @param[in]      row: start row of character
  * @param[in]      col: start column of character
  * @param[in]      chr: the character ready to show
  * @retval         none
  */
/**
  * @brief          显示一个字符
  * @param[in]      row: 字符的开始行
  * @param[in]      col: 字符的开始列
  * @param[in]      chr: 字符
  * @retval         none
  */
void OLED_show_char(uint8_t row, uint8_t col, uint8_t chr)
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
                OLED_draw_point(x, y, PEN_WRITE);
            else
                OLED_draw_point(x, y, PEN_CLEAR);

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
  * @brief          show a character string
  * @param[in]      row: row of character string begin
  * @param[in]      col: column of character string begin
  * @param[in]      chr: the pointer to character string
  * @retval         none
  */
/**
  * @brief          显示一个字符串
  * @param[in]      row: 字符串的开始行
  * @param[in]      col: 字符串的开始列
  * @param[in]      chr: 字符串
  * @retval         none
  */
void OLED_show_string(uint8_t row, uint8_t col, uint8_t *chr)
{
    uint8_t n =0;

    while (chr[n] != '\0')
    {
        OLED_show_char(row, col, chr[n]);
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
  * @brief          send the data of gram to oled sreen
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送数据到OLED的GRAM
  * @param[in]      none
  * @retval         none
  */
void OLED_refresh_gram(void)
{	
	uint8_t i;
	uint16_t j;
		
	if(BufFinshFlag == 0)
	{
		for(i = 0; i < 8; i ++ )
		{
			OLED_set_pos(0, i);
			for(j = 0;j < 128; j ++)
			{
					OLED_GRAMbuf[i][j] = OLED_GRAM[j][i];  //OLED_GRAM[128][8]
			}
		}
		BufFinshFlag = 1;
		oled_module.HW_I2C_Transmit(OLED_CMDbuf[0],4);
	}
}







