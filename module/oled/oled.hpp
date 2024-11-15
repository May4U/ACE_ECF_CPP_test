#ifndef __OLED_HPP
#define __OLED_HPP

#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "bsp_i2c.hpp"
#include "bsp_dwt.hpp"
#include "oledfont.h"

#ifdef __cplusplus
}

#endif


// the I2C address of oled
#define OLED_I2C_ADDRESS    0x78

//the resolution of oled   128*64
#define MAX_COLUMN      128
#define MAX_ROW         64

#define X_WIDTH         MAX_COLUMN
#define Y_WIDTH         MAX_ROW

#define OLED_CMD        0x00
#define OLED_DATA       0x01

#define CHAR_SIZE_WIDTH 6
#define CHAR_SIZE_HIGHT 12

namespace OLED_n
{
    typedef enum
    {
        Hardware = 0,
        Software,
    } oled_mode_e;

    typedef enum
    {
        PEN_CLEAR = 0x00,
        PEN_WRITE = 0x01,
        PEN_INVERSION= 0x02,
    }pen_typedef;

    // typedef struct
    // {
    //     uint8_t OLED_GRAM[128][8];
    //     uint8_t OLED_GRAMbuf[8][128];
    //     uint8_t OLED_CMDbuf[8][4] = {0};
    //     uint8_t I2C_MemTxFinshFlag = 1;
    //     uint8_t CountFlag = 0; 
    //     uint8_t BufFinshFlag = 0; 

    // } oled_init_t;

    

    /**
     * @brief 
     * 
     */
    class OLED_c : public BSP_I2C_n::BSP_I2C_c
    {
    public:
        /*OLED对外函数*/
        void OLED_Init(void);
        void OLED_Clear(void);
        void OLED_Show_Char(uint8_t row, uint8_t col, uint8_t chr);
        void OLED_Show_String(uint8_t row, uint8_t col, uint8_t *chr);
        void OLED_Show_Num(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
        void OLED_Show_SignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
        void OLED_Show_HexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
        void OLED_Show_BinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
        
        //BSP_I2C_n::BSP_I2C_c oled_i2c_instance_;
        OLED_c(BSP_I2C_n::HW_I2C_Config_s HW_I2C_Config);
        OLED_c(BSP_I2C_n::SW_I2C_Config_s SW_I2C_Config);
        OLED_c();
        ~OLED_c();

        uint8_t OLED_GRAM[128][8];
        uint8_t OLED_GRAMbuf[8][128];
        uint8_t OLED_CMDbuf[8][4] = {0};
        uint8_t I2C_MemTxFinshFlag = 1;
        uint8_t CountFlag = 0; 
        uint8_t BufFinshFlag = 0; 
    protected:
        //oled_init_t oled_init_;
        static oled_mode_e oled_mode_;
        /*硬件OLED库*/
        
        void HW_Operate_Gram(pen_typedef pen);
        void HW_Refresh_Gram(void);
        void HW_Set_Pos(uint8_t x, uint8_t y);
        void HW_Draw_Point(int8_t x, int8_t y, pen_typedef pen);
        void HW_Draw_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen);
        void HW_Show_Char(uint8_t row, uint8_t col, uint8_t chr);
        void HW_Show_String(uint8_t row, uint8_t col, uint8_t *chr);
        uint32_t HW_Pow(uint8_t m, uint8_t n);
        void HW_Show_Num(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
        void HW_Init(void);
        /*模拟OLED库*/
        void SW_I2C_Init(void);
        void SW_I2C_Start(void);
        void SW_I2C_Stop(void);
        void SW_I2C_SendByte(uint8_t Byte);
        void SW_WriteCommand(uint8_t Command);
        void SW_WriteData(uint8_t Data);
        void SW_SetCursor(uint8_t Y, uint8_t X);
        uint32_t SW_Pow(uint32_t X, uint32_t Y);
        void SW_Clear(void);

        void SW_Init(void);
        void SW_Show_Char(uint8_t Line, uint8_t Column, char Char);
        void SW_Show_String(uint8_t Line, uint8_t Column, uint8_t *String);
        void SW_Show_Num(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);


    };

    

    extern void OLED_init(void);

    uint32_t oled_pow(uint8_t m, uint8_t n);
    uint8_t check_num_len(uint32_t num);
    void OLED_show_num(uint8_t x, uint8_t y, uint32_t num, uint8_t mode, uint8_t len);
    void OLED_show_floatnum(uint8_t x, uint8_t y, float num, uint8_t mode);

    extern void OLED_display_on(void);

    extern void OLED_display_off(void);

    extern void OLED_printf(uint8_t row, uint8_t col, const char *fmt,...);

    extern void OLED_refresh_gram(void);
}



#endif