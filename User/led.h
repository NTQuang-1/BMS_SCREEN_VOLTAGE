#ifndef __LED_H_
#define __LED_H_
#include <ch32v00x_gpio.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define LED_FLASH_ADDR 0x08003FE0

  typedef enum {LED_ON, LED_OFF, LED_EFFECT} led_status_t;
  typedef enum {LCD_ON,LCD_OFF             } lcd_status_t;

  void    LED_INIT             (void);
  void    SaveLedLevelToFlash  (uint8_t level);
  uint8_t ReadLedLevelFromFlash(void);

#ifdef __cplusplus
}
#endif
#endif
