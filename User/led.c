/**
 * @file led.c
 * @brief Some function initial and controller led
 * @version 0.1
 * @date 22/11/2025
 * @copyright AZ Electronic
 */

#include "led.h"

void LED_INIT(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  /* Enable clocks for GPIOD */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

  /* Configure GPIOD Pin 6 as output push-pull */
  gpio_init.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOD, &gpio_init);

  GPIOD->BCR = GPIO_Pin_5 | GPIO_Pin_6;
}

void SaveLedLevelToFlash(uint8_t level)
{
  FLASH_Unlock();
  FLASH_ErasePage(LED_FLASH_ADDR);
  FLASH_ProgramHalfWord(LED_FLASH_ADDR, level);
  FLASH_Lock();
}

uint8_t ReadLedLevelFromFlash(void)
{
  return *(uint8_t *)LED_FLASH_ADDR;
}