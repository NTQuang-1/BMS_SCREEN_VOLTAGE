/**
 * @file time.c
 * @brief Some function relate create timer
 * @version 0.1
 * @date 22/11/2025
 * @copyright AZ Electronic
 */

#include "time.h"

/**
 * @brief TIM 1 Initialize
 *
 * @param freq : Frequency Interrupt 1ms
 */
void TIM1_INIT(void)
{
  NVIC_InitTypeDef NVIC_InitStructure = {0};
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};

  /* Enable TIM1 clock */
  TIM_DeInit(TIM1);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* Configure TIM1 time base */
  TIM_TimeBaseStructure.TIM_Prescaler = 48 - 1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Enable TIM1 update interrupt */
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);

  /* NVIC configuration */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_Cmd(TIM1, ENABLE); // start timer interrupt
}

/**
 * @brief TIM 2 Initialize
 *
 * @param freq : Frequency Interrupt
 */

void TIM2_INIT(void)
{
  TIM_OCInitTypeDef TIM_OCInitStructure = {0};

  /* Enable clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);

  /* Remap PD6 -> TIM2_CH3, PD5 -> TIM2_CH4 */
  AFIO->PCFR1 &= ~(uint32_t)0x00000300;
  AFIO->PCFR1 |= (uint32_t)0x0000300;

  /* Configure PD5 PD6 as alternate function push-pull with 50MHz */
  GPIOD->CFGLR &= ~(uint32_t)0x0FF00000;
  GPIOD->CFGLR |= (uint32_t)0x0BB00000;

  /* Timer base configuration */
  TIM2->CTLR1 &= ~(uint16_t)0x0370; // Count up, TIM div 1
  TIM2->PSC = 48 - 1;               // Prescaler
  TIM2->ATRLR = 1000 - 1;           // Period
  TIM2->SWEVGR = (uint16_t)0x0001;

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM2->CTLR1 |= TIM_ARPE; // Auto-reload preload enable

  /* Start timer */
  TIM2->CTLR1 = TIM_CEN;
}
