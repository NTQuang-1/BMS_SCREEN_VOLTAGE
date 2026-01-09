
/**
 * @file    : ntc.c
 * @date    : 04/12/2024
 * @details : Source file for read and handle thermitor YR-91191A(The special LCD Display).
 */
/*******************************************************************************
 * Include.
*******************************************************************************/
#include "ntc.h"
#include "stddef.h"

/*******************************************************************************
 * Definition.
*******************************************************************************/
#define NTC_103_3950

/*******************************************************************************
 * Prototype.
*******************************************************************************/
int16_t arr_find_apxm_val(const uint16_t *arr, const uint16_t val, uint16_t length);

/*******************************************************************************
 * Variable.
*******************************************************************************/
#ifdef  NTC_103_3950
#define NUM_OF_NTC_REF  (24U)
static const uint16_t ntc_val_ref[NUM_OF_NTC_REF] = 
{
    51820, 40450,  /* -10 -> -5*/
    31770, 24940,  /* 0 -> 5℃*/
    19680, 15620,  /* 10 - 15℃*/
    12470, 10000,  /* 20 - 25℃*/
    8064,  6538,   /* 30 - 35℃*/
    5327,  4363,   /* 40 - 45℃*/
    3592,  2972,   /* 50 - 55℃*/
    2472,  2066,   /* 60 - 65℃*/
    1735,  1465,   /* 70 - 75℃*/
    1243,  1059,   /* 80 - 85℃*/
    908,   781,    /* 90 - 95℃*/
    674,   585,    /* 100 - 105℃*/
};

/*******************************************************************************
 * Code.
*******************************************************************************/
int16_t NTC_TempAnls(uint32_t ntc_res)
{
    uint32_t temperature;   /* Temperature.*/
    uint16_t ntc_res_in_rg; /* In ranger of ntc resistancw*/
    uint16_t top_ref_ranger;
    uint16_t bot_ref_ranger;

    if (ntc_res > ntc_val_ref[0])
    {
        temperature = -9;
    }
    else if (ntc_res < ntc_val_ref[23])
    {
        temperature = 105;
    }
    else
    {
        /* Find ntc res ranger*/
        ntc_res_in_rg = arr_find_apxm_val(ntc_val_ref, ntc_res, NUM_OF_NTC_REF);
        bot_ref_ranger = ntc_val_ref[ntc_res_in_rg];
        top_ref_ranger = ntc_val_ref[ntc_res_in_rg + 1];
        temperature = (ntc_res_in_rg * 5 - 10) + 5*(bot_ref_ranger - ntc_res) / (bot_ref_ranger - top_ref_ranger);
    }

    return temperature;
}

/**
 * @brief Find approximate member in array follow the special parameter. return 
 *        index of the approximate member or -1 if process is fail.
 * 
 * @note  keep the array is Arrange from largest to smallest.
 * @param arr
 * @param val
 * @param length
 * @retval 
 */
int16_t arr_find_apxm_val(const uint16_t *arr, const uint16_t val, uint16_t length)
{
    uint16_t low;
    uint16_t mid;
    uint16_t high;

    /* Check the parameter.*/
    if ((NULL == arr) || (0 == length))
    {
        return -1;
    }

    /* base param.*/
    low = 0;
    high = length - 1;
    mid = (low + high) / 2;

    /* Find approximate member*/
    while (mid != low)
    {
        if (arr[mid] >= val)
        {
            /* Check data from mid to high.*/
            low = mid;
            mid = (low + high) / 2;
        }
        else
        {
            /* Check data from low to mid.*/
            high = mid;
            mid = (low + high) / 2;
        }
    }
    
    return mid;
}


#endif /*NTC_103_3950*/
