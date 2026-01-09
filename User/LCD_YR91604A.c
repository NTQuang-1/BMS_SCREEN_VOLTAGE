

/*************************************************************************************************************************************
 * Include
 *************************************************************************************************************************************/
#include "LCD_YR91604A.h"
#include "tm1621.h"

/*************************************************************************************************************************************
 * Defintions.
 *************************************************************************************************************************************/
#define LCD_GPIO (GPIOC)
#define LCD_BACK_LIGTH_PIN (GPIO_Pin_0)
/*************************************************************************************************************************************
 * Prototype.
 *************************************************************************************************************************************/

/*************************************************************************************************************************************
 * Variable.
 *************************************************************************************************************************************/
const uint8_t gc_seg7_code1[11] = {0xAFu, 0xA0u, 0xCBu, 0xE9u, 0xE4u, 0x6Du, 0x6Fu, 0xA8u, 0xEFu, 0xEDu, 0x00u};
const uint8_t gc_seg7_code2[11] = {0xF5u, 0x05u, 0xB6u, 0x97u, 0x47u, 0xD3u, 0xF3u, 0x85u, 0xF7u, 0xD7u, 0x00u};
static uint8_t gp_lcd_dat[11];

/*************************************************************************************************************************************
 * Code.
 *************************************************************************************************************************************/
/**
 * @brief Shutdown lcd.
 */
void LCD_Shutdown(void)
{
  TM1621_SendCmd(TM1621_CMD_SYS_DIS);
}

/**
 * @brief Turn on LCD.
 */
void LCD_On(void)
{
  TM1621_SendCmd(TM1621_CMD_SYS_EN);
  TM1621_SendCmd(TM1621_CMD_LCD_ON);
}

/**
 * @brief   Initializes the LCD display.
 *          This function configures the LCD hardware and prepares it for use.
 */
void LCD_Init(void)
{
  /* Initializes the TM1621 control pin.*/
  GPIOC->CFGLR &= ~(uint32_t)0x0F;
  GPIOC->CFGLR |= (uint32_t)0x01;   // Out_PP, LCD_BACK_LIGTH_PIN, speed 10MHz
  GPIOC->BCR = LCD_BACK_LIGTH_PIN;  // GPIO write level 0
  AFIO->PCFR1 &= ~(uint32_t)0x8000; // remap PA1_2 disable

  TM16_Init();

  TM1621_SendCmd(TM1621_CMD_SYS_EN);
  TM1621_SendCmd(TM1621_CMD_BIAS1_3_4P);
}

void LCD_Clear(void)
{
  /* Contain the clear value, to wite*/
  uint8_t clr_dat_buff[16] = {0x00};

  TM1621_SendMulDat(0x00, clr_dat_buff, 32);
}

/**
 * @brief
 */
void LCD_BackLightCmd(FunctionalState new_stat)
{
  if (new_stat)
    LCD_GPIO->BSHR = LCD_BACK_LIGTH_PIN; // level 1
  else
    LCD_GPIO->BCR = LCD_BACK_LIGTH_PIN; // level 0
}

/**
 * @brief   Show battery voltage follow special parameter.
 * @param   bat_vol : battery voltage in (100 mV).
 * @return  None.
 */
void LCD_ShwBatVol(uint16_t bat_vol)
{
  uint8_t seg7_val;

  /* X0,1 voltage.*/
  seg7_val = bat_vol % 10;
  gp_lcd_dat[2] = gc_seg7_code1[seg7_val] | 0x10;

  /* X0,1 voltage.*/
  seg7_val = (bat_vol / 10) % 10;
  gp_lcd_dat[1] = gc_seg7_code1[seg7_val] | 0x10;

  /* X0,1 voltage.*/
  seg7_val = (bat_vol / 100) % 10;
  gp_lcd_dat[0] = gc_seg7_code1[seg7_val] | 0x10;

  TM1621_SendMulDat(0x00, gp_lcd_dat, 0x06);
}

/**
 * @brief   Show battery temperature on LCD.
 * @param   tempe : Battery temperature in degrees Celsius.
 * @return  None.
 */
void LCD_ShowTempe(int16_t tempe)
{
  uint8_t seg7_val;

  if (tempe >= 0)
  {
    /* X0,1 voltage.*/
    seg7_val = tempe % 10;
    gp_lcd_dat[3] = gc_seg7_code2[seg7_val] | 0x08;

    /* X0,1 voltage.*/
    seg7_val = (tempe / 10) % 10;
    gp_lcd_dat[4] = gc_seg7_code2[seg7_val];

    if (tempe >= 100)
    {
      gp_lcd_dat[4] |= 0x08u;
    }
  }
  else
  {
    gp_lcd_dat[3] |= 0x20u;
    if (tempe < -9)
    {
      gp_lcd_dat[4] = gc_seg7_code2[10] | 0x08u;
    }
    else
    {
      gp_lcd_dat[4] = gc_seg7_code2[-(tempe)] | 0x08u;
    }
  }
  gp_lcd_dat[7] |= 0x08;
  TM1621_SendMulDat(26, gp_lcd_dat + 3, 0x04);
}

/**
 * @brief   Show the specified battery percentage on the LCD.
 * @param   bat_prc : Battery percentage (0 to 100).
 * @param   char_percent_en : Character to indicate percentage (e.g., '%').
 * @return  None.
 */
void LCD_ShowBatPrc(uint8_t bat_prc, uint8_t char_percent_en)
{

  if (char_percent_en)
  {
    uint8_t seg7_val;
    /* X0,1 voltage.*/
    seg7_val = bat_prc % 10;
    gp_lcd_dat[5] = gc_seg7_code2[seg7_val] | 0x08u;

    /* X0,1 voltage.*/
    seg7_val = (bat_prc / 10) % 10;
    gp_lcd_dat[6] = gc_seg7_code2[seg7_val];

    if (bat_prc >= 100)
    {
      gp_lcd_dat[6] |= 0x08u;
    }
  }
  else
  {
    gp_lcd_dat[5] = gc_seg7_code2[11];
    gp_lcd_dat[6] = gc_seg7_code2[bat_prc % 10];
  }

  TM1621_SendMulDat(15, gp_lcd_dat + 5, 0x04);
}

void LCD_ShowBatLev(uint8_t bat_lev, uint8_t bat_frame_en)
{
  uint8_t new7 = 0x00, new8 = 0x00, new9 = 0x00;

  if (bat_lev > 10)
  {
    new7 = 0x07u;
    new8 = 0x0Fu;
    new9 = 0x0Fu;
  }
  else
  {
    if (bat_lev == 0)
    {
      new7 = 0x00u;
      new8 = 0x00u;
      new9 = 0x00u;
    }
    else if (bat_lev < 4)
    {
      new7 = (uint8_t)((0xF0u >> (bat_lev + 1)) & 0x07u);
      new8 = 0x00u;
      new9 = 0x00u;
    }
    else if (bat_lev < 8)
    {
      new7 = 0x07u;
      new8 = (uint8_t)((0x1Eu >> (8 - bat_lev)) & 0x0Fu);
      new9 = 0x00u;
    }
    else
    {
      new7 = 0x07u;
      new8 = 0x0Fu;
      new9 = (uint8_t)((0x78u >> (bat_lev - 8)) & 0x7Eu);
    }
  }

  gp_lcd_dat[7] = (gp_lcd_dat[7] & (uint8_t)(~0x07u)) | (new7 & 0x07u);
  gp_lcd_dat[8] = (gp_lcd_dat[8] & (uint8_t)(~0x0Fu)) | (new8 & 0x0Fu);
  gp_lcd_dat[9] = (gp_lcd_dat[9] & (uint8_t)(~0x7Eu)) | (new9 & 0x7Eu);

  if (bat_frame_en)
    gp_lcd_dat[9] |= 0x01u;
  else
    gp_lcd_dat[9] &= ~0x01u;

  TM1621_SendSingleDat(30, gp_lcd_dat[7]);
  TM1621_SendSingleDat(19, gp_lcd_dat[8]);
  TM1621_SendSingleDat(13, gp_lcd_dat[9]);
}

/**
 * @brief Show special item.
 *
 * @param item :    LCD_SPC_ITEM_OV_HEAT
                    LCD_SPC_ITEM_CHG
                    LCD_SPC_ITEM_LOW
                    LCD_SPC_ITEM_CAP_STR
                    LCD_SPC_ITEM_TEMPE_STR
 * @param show
 */
void LCD_ShowSpcItem(uint8_t item, FunctionalState new_state)
{
  if (0 != (item & 0x0fu))
  {
    if (new_state)
    {
      gp_lcd_dat[10] |= item & 0x0fu;
    }
    else
    {
      gp_lcd_dat[10] &= ~(item & 0x0fu);
    }

    TM1621_SendSingleDat(14, gp_lcd_dat[10]);
  }
}