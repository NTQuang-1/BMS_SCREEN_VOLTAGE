

#include "ch32v00x_gpio.h"
#include "ch32v00x_adc.h"
#include "meansure.h"
#include "ntc.h"

#define ADC_V_in_PIN   (GPIO_Pin_4)
#define ADC_V_in_PORT  (GPIOC)

#define ADC_NTC_PIN    (GPIO_Pin_4)
#define ADC_NTC_PORT   (GPIOD)

#define ADC_NTC_V_PIN  (GPIO_Pin_7)
#define ADC_NTC_V_PORT (GPIOC)

#define CALIB_STORE_ADDR 0x08003FC0
typedef struct
{
  uint16_t v_ref;
  int16_t v_offset;
  float gain;
  float mul;
} BatVoltCalib_Typedef;

BatVoltCalib_Typedef g_bat_volt_calib = {.v_ref = 5000, .v_offset = 0, .gain = 4.16f, .mul = 20.335f};

float g_bat_volt_mul;

uint32_t MeansureInit(void)
{
  GPIO_InitTypeDef adc_pin_config;
  ADC_InitTypeDef adc_config;
  uint16_t tmp[6];

  /* Config pins as adc mode.*/
  adc_pin_config.GPIO_Mode = GPIO_Mode_AIN;
  adc_pin_config.GPIO_Pin = ADC_V_in_PIN;
  adc_pin_config.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(ADC_V_in_PORT, &adc_pin_config);

  adc_pin_config.GPIO_Mode = GPIO_Mode_AIN;
  adc_pin_config.GPIO_Pin = ADC_NTC_PIN;
  adc_pin_config.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(ADC_NTC_PORT, &adc_pin_config);

  adc_pin_config.GPIO_Mode = GPIO_Mode_Out_PP;
  adc_pin_config.GPIO_Pin = ADC_NTC_V_PIN;
  adc_pin_config.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(ADC_NTC_V_PORT, &adc_pin_config);

  /* Config adc.*/
  adc_config.ADC_ContinuousConvMode = DISABLE;
  adc_config.ADC_DataAlign = ADC_DataAlign_Right;
  adc_config.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  adc_config.ADC_Mode = ADC_Mode_Independent;
  adc_config.ADC_NbrOfChannel = 1;
  adc_config.ADC_ScanConvMode = DISABLE;

  ADC_Init(ADC1, &adc_config);
  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  while (ADC_GetResetCalibrationStatus(ADC1))
    ;
  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1))
    ;

  tmp[0] = *(uint16_t *)0x08003FC0;
  tmp[1] = *(uint16_t *)(0x08003FC0 + 2);
  tmp[2] = *(uint16_t *)(0x08003FC0 + 4);
  tmp[3] = *(uint16_t *)(0x08003FC0 + 6);
  tmp[4] = *(uint16_t *)(0x08003FC0 + 8);
  tmp[5] = *(uint16_t *)(0x08003FC0 + 10);

  if (tmp[0] == 0xffff)
  {
    /* Not calib*/
  }
  else
  {
    g_bat_volt_calib = *(BatVoltCalib_Typedef *)(&tmp);
  }

  // g_bat_volt_mul = ((float)g_bat_volt_calib.v_ref * g_bat_volt_calib.gain) / 1023.0f;
  g_bat_volt_mul = g_bat_volt_calib.mul;
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_73Cycles);
  Delay_Ms(10);
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    ;
  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  if (ADC_GetConversionValue(ADC1) < 100)
  {
    return MEANSURE_SRC_External;
  }
  else
  {
    return MEANSURE_SRC_Internal;
  }
}

/**
 * @brief Calibrations for read battery voltage follow special param.
 *
 * @param calib_vref
 * @param calib_offset
 * @param calib_gain
 */
void MeansureCalibBatVolt(uint16_t calib_vref, int16_t calib_offset, float calib_gain, float mul)
{
  uint16_t *tmp_buff;

  if (calib_vref != 0)
  {
    g_bat_volt_calib.gain = calib_gain;
    g_bat_volt_calib.v_offset = calib_offset;
    g_bat_volt_calib.v_ref = calib_vref;
  }

  g_bat_volt_calib.mul = mul;

  tmp_buff = (uint16_t *)(&g_bat_volt_calib);

  FLASH_ErasePage(CALIB_STORE_ADDR);
  FLASH_Unlock();
  FLASH_ProgramHalfWord(CALIB_STORE_ADDR, (uint16_t)tmp_buff[0]);
  FLASH_ProgramHalfWord(CALIB_STORE_ADDR + 2, (uint16_t)tmp_buff[1]);
  FLASH_ProgramHalfWord(CALIB_STORE_ADDR + 4, (uint16_t)tmp_buff[2]);
  FLASH_ProgramHalfWord(CALIB_STORE_ADDR + 6, (uint16_t)tmp_buff[3]);
  FLASH_ProgramHalfWord(CALIB_STORE_ADDR + 8, (uint16_t)tmp_buff[4]);
  FLASH_ProgramHalfWord(CALIB_STORE_ADDR + 10, (uint16_t)tmp_buff[5]);
  FLASH_Lock();
}

void MeansureAutoCalid(void)
{
  uint16_t adc_sum = 0;
  uint16_t idx;
  float adc_val;

  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_73Cycles);
  Delay_Ms(10);
  for (idx = 0; idx < 50; idx++)
  {
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
      ;
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    adc_sum += ADC_GetConversionValue(ADC1);
  }

  adc_val = (float)adc_sum / 50.f;
  g_bat_volt_mul = 12000 / adc_val;
  MeansureCalibBatVolt(0, 0, 0, g_bat_volt_mul);

  adc_sum = MeansureBatVolt();
  g_bat_volt_calib.v_offset = 12000 - adc_sum;
}

/**
 * @brief Meansure battery voltage & return value at mV
 *
 * @return uint16_t
 */
int MeansureBatVolt(void)
{
  uint16_t adc_sum = 0;
  int bat_volt;
  uint16_t idx;
  float adc_val;

  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_73Cycles);
  Delay_Ms(10);
  for (idx = 0; idx < 50; idx++)
  {
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
      ;
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    adc_sum += (uint16_t)ADC1->RDATAR;
  }

  adc_val = (float)adc_sum / 50.f;
  bat_volt = (int)(g_bat_volt_mul * adc_val) + g_bat_volt_calib.v_offset;

  return bat_volt;
}

int16_t MeansureTempe(void)
{
  uint16_t adc_sum = 0;
  uint32_t ntc_res;
  int16_t tempe;
  uint16_t idx;
  float adc_val;

  GPIO_WriteBit(ADC_NTC_V_PORT, ADC_NTC_V_PIN, Bit_SET);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_73Cycles);
  Delay_Ms(10);
  for (idx = 0; idx < 20; idx++)
  {
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
      ;
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    adc_sum += ADC_GetConversionValue(ADC1);
  }

  adc_val = (float)adc_sum / 20.f;
  ntc_res = ((uint32_t)(adc_val) * 10000) / (1023 - adc_val);
  tempe = NTC_TempAnls(ntc_res);

  GPIO_WriteBit(ADC_NTC_V_PORT, ADC_NTC_V_PIN, Bit_RESET);
  return tempe;
}