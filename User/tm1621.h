/**
 * @file    : lcd_tm1621.h
 * @date    : 30/11/2024
 * @details : Header file for controling LCD using TM1621 familiar driver IC.
 */

#ifndef TM1621_H
#define TM1621_H

/*******************************************************************************
 * Include.
*******************************************************************************/
#include "ch32v00x_gpio.h"

/*******************************************************************************
 * Definitions.
*******************************************************************************/
#define TM1621_CMD_SYS_DIS      ((uint16_t)0x0000u) /** Shutdown system oscillator and LCD bias generations.*/
#define TM1621_CMD_SYS_EN       ((uint16_t)0x0002u) /** Turn on system oscilator.*/
#define TM1621_CMD_LCD_OFF      ((uint16_t)0x0004u) /** Turn off the LCD bias generations.*/
#define TM1621_CMD_LCD_ON       ((uint16_t)0x0006u) /** Turn on the LCD bisa generations.*/
#define TM1621_CMD_XTAL_32K     ((uint16_t)0x0028u) /** System clock source is crytal oscillator.*/
#define TM1621_CMD_RC_256K      ((uint16_t)0x0030u) /** System clock source is on-chip RC oscillator.*/
#define TM1621_CMD_BIAS1_2_2P   ((uint16_t)0x0040u) /** LDC bias 1/2 port optional as 2,3,4 public port. */
#define TM1621_CMD_BIAS1_2_3P   ((uint16_t)0x0048u)
#define TM1621_CMD_BIAS1_2_4P   ((uint16_t)0x0050u)
#define TM1621_CMD_BIAS1_3_2P   ((uint16_t)0x0042u) /** LDC bias 1/3 port optional as 2,3,4 public port. */
#define TM1621_CMD_BIAS1_3_3P   ((uint16_t)0x004au)
#define TM1621_CMD_BIAS1_3_4P   ((uint16_t)0x0052u)
#define TM1621_CMD_TOPT         ((uint16_t)0x01c0u) /** Test mode.*/
#define TM1621_CMD_TNORMAL      ((uint16_t)0x01c4u) /** Normal mode. */
/*******************************************************************************
 * Private prototype.
*******************************************************************************/
void TM16_Init(void);
void TM1621_SendCmd(const uint16_t cmd);
void TM1621_SendSingleDat(const uint16_t ram_add, const uint16_t dat);
void TM1621_SendMulDat(const uint16_t ram_add, uint8_t *p_dat, uint8_t num_data);
#endif /*LCD_TM1621_H*/

