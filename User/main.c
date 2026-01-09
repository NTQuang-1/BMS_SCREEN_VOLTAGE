#include "tm1621.h"
#include "LCD_YR91604A.h"
#include "meansure.h"
#include "time.h"
#include "led.h"

/** Const & globals ----------------------------------------------------------*/
const uint16_t gc_3_2_volt_ref[12] = {2500, 3000, 3100, 3140, 3180, 3220, 3250, 3280, 3310, 3340, 3370, 3400}; // 10%->100%
uint16_t bat_volt_ref[11];

static volatile uint16_t sys_stick = 0;
static volatile uint16_t led_dutyCycleNow = 0, led_dutyCycleMax = 0;
static volatile uint8_t led_level = 1;
led_status_t led_status = LED_OFF, led_status_in_mode_led_on_off = LED_OFF;
lcd_status_t lcd_status = LCD_OFF;

uint8_t remain_battery;
int bat_volt;
int tempe;
uint8_t idx;
uint32_t mean_src;

/* Button handling */
static volatile uint8_t button_pressed = 0;
static volatile uint32_t press_ms = 0;
static volatile uint8_t press_pending = 0;
static volatile uint32_t press_duration_ms = 0;

/* Hold detection */
static volatile uint8_t hold3s_done = 0;
static volatile uint8_t hold5s_done = 0;

/* Blink tick for led_on_off */
static volatile uint32_t blink_tick_ms = 0;

/** Prototype -----------------------------------------------------------------*/
static void Button_Init(void);
uint8_t CalibChecking(void);
uint8_t pro_remain_bat(uint16_t bat_volt);
void EnterSleep(void);
static inline void TIM2_SetDuty(uint16_t dutyCycle);

/** Implementation -----------------------------------------------------------*/

int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  SystemCoreClockUpdate();
  Delay_Init();

  RCC_ADCCLKConfig(RCC_PCLK2_Div4);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE);

  LCD_Init();
  LCD_Clear();
  TM1621_SendCmd(TM1621_CMD_LCD_ON);
  lcd_status = LCD_ON;

  Button_Init();
  LED_INIT();
  TIM2_INIT();
  TIM1_INIT(); // Start 1ms interrupt

  led_level = ReadLedLevelFromFlash();

  mean_src = MeansureInit();

  for (idx = 0; idx < 11; idx++)
    bat_volt_ref[idx] = gc_3_2_volt_ref[idx] * 4;

  if (MEANSURE_SRC_Internal == mean_src)
  {
    if (CalibChecking())
    {
      MeansureAutoCalid();
      while (1)
      {
        bat_volt = MeansureBatVolt();
        LCD_BackLightCmd(ENABLE);
        LCD_ShwBatVol(bat_volt / 100);
        LCD_ShowTempe(bat_volt % 100);
        Delay_Ms(1000);
      }
    }
    else
    {
      bat_volt = (MeansureBatVolt() + MeansureBatVolt() + MeansureBatVolt() + MeansureBatVolt()) / 4;
      led_dutyCycleMax = (bat_volt > 12) ? (uint16_t)((12000.0f / bat_volt) * 1000.0f) : 1000;
      tempe = MeansureTempe();
      remain_battery = pro_remain_bat(bat_volt);

      LCD_BackLightCmd(ENABLE);
      LCD_ShowSpcItem(LCD_SPC_ITEM_CAP_STR, ENABLE);
      LCD_ShwBatVol(bat_volt / 100);
      LCD_ShowTempe(tempe);
      LCD_ShowBatLev(remain_battery / 10, ENABLE);
      LCD_ShowBatPrc(remain_battery, ENABLE);
    }
  }

  uint32_t last_blink_toggle_ms = 0;

  while (1)
  {
    if (press_pending)
    {
      __disable_irq();
      uint32_t dur = press_duration_ms;
      press_pending = 0;
      press_duration_ms = 0;
      __enable_irq();

      /* Short press: increase level cyclic */
      if (dur >= 30 && dur < 1000)
      {
        if (lcd_status == LCD_OFF)
        {
          LCD_Init();
          TM1621_SendCmd(TM1621_CMD_LCD_ON);
          LCD_Clear();
          lcd_status = LCD_ON;
          LCD_BackLightCmd(ENABLE);
          LCD_ShowSpcItem(LCD_SPC_ITEM_CAP_STR, ENABLE);
          LCD_ShwBatVol(bat_volt / 100);
          LCD_ShowTempe(tempe);
          LCD_ShowBatLev((led_level * 2), ENABLE);
        }

        if (led_status == LED_ON)
        {
          led_level = (led_level < 5) ? (led_level + 1) : 1;
          led_dutyCycleNow = (led_dutyCycleMax * led_level) / 5;
          LCD_ShowBatLev((led_level * 2), ENABLE);
          LCD_ShowBatPrc(led_level, DISABLE);
          TIM2_SetDuty(led_dutyCycleNow);
        }
        else if (led_status == LED_EFFECT)
        {
          led_dutyCycleNow = (led_dutyCycleMax * led_level) / 5;
          LCD_ShowBatLev((led_level * 2), ENABLE);
          LCD_ShowBatPrc(led_level, DISABLE);
        }
      }
    }

    /* Led on_off blink */
    if (led_status == LED_EFFECT)
    {
      uint32_t cur = blink_tick_ms;
      if ((cur - last_blink_toggle_ms) >= 500)
      {
        last_blink_toggle_ms = cur;
        if (led_status_in_mode_led_on_off == LED_ON)
        {
          led_status_in_mode_led_on_off = LED_OFF;
          LCD_ShowBatLev((led_level * 2), ENABLE);
          LCD_ShowBatPrc(led_level, DISABLE);
          TIM2_SetDuty(led_dutyCycleNow);
        }
        else
        {
          led_status_in_mode_led_on_off = LED_ON;
          TIM2_SetDuty(0);
        }
      }
    }
    Delay_Ms(10);
  }
}

/*--------------------------------- Button & Hold ----------------------------------*/

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line1);

  if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_SET) // Rising
    button_pressed = 1;
  else // Falling
  {
    button_pressed = 0;
    hold3s_done = 0;
    hold5s_done = 0;
    if (press_ms != 0)
    {
      press_duration_ms = press_ms;
      press_ms = 0;
      press_pending = 1;
    }
  }
}

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void)
{
  if (button_pressed)
  {
    press_ms++;
    sys_stick = 0;

    /* 3s hold */
    if (press_ms >= 1000 && !hold3s_done)
    {
      hold3s_done = 1;
      if (led_status == LED_OFF)
      {
        led_status = LED_ON;
        led_dutyCycleNow = (led_dutyCycleMax * led_level) / 5;
        TIM2_SetDuty(led_dutyCycleNow);
        LCD_ShowBatLev(led_level * 2, ENABLE);
        LCD_ShowBatPrc(led_level, DISABLE);
      }
      else
      {
        led_status = LED_OFF;
        TIM2_SetDuty(0);
        LCD_Shutdown();
        EnterSleep();
      }
    }

    /* 5s hold */
    if (press_ms >= 3000 && !hold5s_done)
    {
      hold5s_done = 1;
      led_status = LED_EFFECT;
      led_status_in_mode_led_on_off = LED_OFF;
      led_dutyCycleNow = (led_dutyCycleMax * led_level) / 5;
      TIM2_SetDuty(0);
    }
  }
  else
  {
    sys_stick++;
    if (sys_stick > 16000)
      sys_stick = 0;
  }

  blink_tick_ms++;

  if (sys_stick > 6000)
  {
    if (lcd_status == LCD_ON)
    {
      LCD_Clear();
      LCD_Shutdown();
      LCD_BackLightCmd(DISABLE);
      lcd_status = LCD_OFF;
    }
    if (led_status == LED_OFF)
      EnterSleep();
  }

  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}

static void Button_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};
  EXTI_InitTypeDef EXTI_InitStructure = {0};
  NVIC_InitTypeDef NVIC_InitStructure = {0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static inline void TIM2_SetDuty(uint16_t dutyCycle) { TIM2->CH3CVR = dutyCycle; }

uint8_t CalibChecking(void)
{
  /*----------REMAP SWO-------------*/
  AFIO->PCFR1 &=~(uint32_t)0x07000000;
  AFIO->PCFR1 |= (uint32_t)0x04000000;

  GPIO_InitTypeDef gpio_cofig = {0};
  gpio_cofig.GPIO_Mode = GPIO_Mode_IPU;
  gpio_cofig.GPIO_Pin = GPIO_Pin_1;
  gpio_cofig.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOD, &gpio_cofig);

  return (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1) == Bit_RESET) ? 1 : 0;
}

uint8_t pro_remain_bat(uint16_t bat_volt)
{
  uint8_t rm_bat = 0, idx = 0;
  float step;

  if (bat_volt >= bat_volt_ref[10])
    rm_bat = 100;
  else if (bat_volt >= bat_volt_ref[0])
  {
    while ((idx < 10) && (bat_volt >= bat_volt_ref[idx]))
      idx++;
    step = (idx == 10) ? (float)(bat_volt_ref[10] - bat_volt_ref[9]) / 9 : (float)(bat_volt_ref[idx] - bat_volt_ref[idx - 1]) / 10;
    rm_bat = (idx - 1) * 10 + (bat_volt - bat_volt_ref[idx - 1]) / step;
  }

  return rm_bat;
}

void EnterSleep(void)
{
  SaveLedLevelToFlash(led_level);

  GPIO_InitTypeDef gpio_config = {0};
  EXTI_InitTypeDef exti_config = {0};

  ADC_Cmd(ADC1, DISABLE);
  TIM_Cmd(TIM2, DISABLE);
  TIM_Cmd(TIM1, DISABLE);

  RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    ;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1, DISABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  gpio_config.GPIO_Mode = GPIO_Mode_IPD;
  gpio_config.GPIO_Pin = GPIO_Pin_All;
  GPIO_Init(GPIOA, &gpio_config);
  GPIO_Init(GPIOC, &gpio_config);
  GPIO_Init(GPIOD, &gpio_config);

  gpio_config.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpio_config.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOA, &gpio_config);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

  exti_config.EXTI_Line = EXTI_Line1;
  exti_config.EXTI_LineCmd = ENABLE;
  exti_config.EXTI_Mode = EXTI_Mode_Interrupt;
  exti_config.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&exti_config);

  EXTI_ClearITPendingBit(EXTI_Line1);
  NVIC_EnableIRQ(EXTI7_0_IRQn);
  PWR_EnterSTANDBYMode(PWR_STANDBYEntry_WFI);
  NVIC_SystemReset();
}
