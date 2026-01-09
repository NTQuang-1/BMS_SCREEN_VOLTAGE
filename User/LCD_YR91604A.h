/**
 * @file    : YR_91191A.h
 * @date    : 30/11/2024
 * @details : Header file for controlling and managing YR-91191A(The special LCD Display).
 */

#ifndef LCD_YR91604A_H
#define LCD_YR91604A_H

/*******************************************************************************
 * Include.
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "ch32v00x.h"

/*******************************************************************************
 * Defintions.
*******************************************************************************/
#define LCD_SPC_ITEM_OV_HEAT        ((uint8_t) 0x01u)
#define LCD_SPC_ITEM_CHG            ((uint8_t) 0x02u)
#define LCD_SPC_ITEM_LOW            ((uint8_t) 0x04u)
#define LCD_SPC_ITEM_CAP_STR        ((uint8_t) 0x08u)

/*******************************************************************************
 * Prototype.
*******************************************************************************/
void LCD_Shutdown(void);
void LCD_On(void);
void LCD_Init(void);
void LCD_Clear(void);
void LCD_BackLightCmd(FunctionalState new_stat);
void LCD_ShwBatVol(uint16_t bat_vol);
void LCD_HideBatVol(void);
void LCD_ShowTempe(int16_t tempe);
void LCD_HideTempe(void);
void LCD_ShowBatLev(uint8_t bat_lev, uint8_t bat_frame_en);
void LCD_HideBatLev(void);
void LCD_ShowBatPrc(uint8_t bat_prc, uint8_t char_percent_en);
void LCD_ShowSpcItem(uint8_t item, FunctionalState new_state);
void LCD_Shwcurr(uint16_t curr);
#endif /*LCD_YR91604A_H*/
