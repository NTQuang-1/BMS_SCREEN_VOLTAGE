/**
 * @file    : lcd_tm1621.c
 * @date    : 30/11/2024
 * @details : Source file for controling LCD using TM1621 familiar driver IC.
 */

/*******************************************************************************
 * Include.
*******************************************************************************/
#include "tm1621.h"
#include "debug.h"

/*******************************************************************************
 * Definitions.
*******************************************************************************/
#define TM1621_GPIO         (GPIOC)
#define TM1621_PIN_CS       (GPIO_Pin_3)
#define TM1621_PIN_WR       (GPIO_Pin_2)
#define TM1621_PIN_DAT      (GPIO_Pin_1)

#define TM1621_SET_CS(NEW_STAT)  (GPIO_WriteBit(TM1621_GPIO, TM1621_PIN_CS, NEW_STAT))
#define TM1621_SET_WR(NEW_STAT)  (GPIO_WriteBit(TM1621_GPIO, TM1621_PIN_WR, NEW_STAT))
#define TM1621_SET_DAT(NEW_STAT) (GPIO_WriteBit(TM1621_GPIO, TM1621_PIN_DAT, NEW_STAT))
#define TM1621_SET_IDLE() (GPIOC->BSHR |= TM1621_PIN_DAT | TM1621_PIN_WR | TM1621_PIN_CS)


// #define TM1621_SET_CS()       (GPIOC->BSHR |= TM1621_PIN_CS)
// #define TM1621_SET_WR()       (GPIOC->BSHR |= TM1621_PIN_WR)
// #define TM1621_SET_DAT()      (GPIOC->BSHR |= TM1621_PIN_DAT)
// #define TM1621_CLEAR_CS()     (GPIOC->BCR |= TM1621_PIN_CS)
// #define TM1621_CLEAR_WR()     (GPIOC->BCR |= TM1621_PIN_WR)
// #define TM1621_CLEAR_DAT()    (GPIOC->BCR |= TM1621_PIN_DAT)


#define TM1621_CMD_Msk      ((uint16_t) 0x0800u)
#define LENGTH_OF_FRAME_CMD ((uint32_t) 0x000cu)
#define TM1621_DAT_Msk      ((uint16_t) 0x1400u)
#define LENGTH_OF_FRAME_DAT ((uint32_t) 0x000du)

#define TM1621_RAM_ADD_Pos  (4u)
#define TM1621_RAM_ADD_Msk  ((uint16_t) (0x003fu << TM1621_RAM_ADD_Pos))
#define TM1621_RAM_DAT_Pos  (0u)
#define TM1621_RAM_DAT_Msk  ((uint16_t) 0x000fu)

// #define TM_DELAY()  Delay_Us(100)

/*******************************************************************************
 * Private prototype.
*******************************************************************************/
//static void SendDat(uint8_t *p_dat, uint length);

/*******************************************************************************
 * Code.
*******************************************************************************/

/**
 * @brief   Initializes the TM1621 LCD driver IC, and necessary peripheral.
 * @return  None.
 */
void TM16_Init(void)
{
    TM1621_GPIO->CFGLR &= ~(uint32_t)0xFFF0;
    TM1621_GPIO->CFGLR |= (uint32_t)0x1110; // Out_PP, TM1621_PIN_CS, TM1621_PIN_WR, TM1621_PIN_DAT, Speed 10MHz.

    /* Setting pin in the idle status.*/
    TM1621_GPIO->BSHR = TM1621_PIN_DAT | TM1621_PIN_WR | TM1621_PIN_CS; // GPIO level 1

    //Delay_Ms(10);
}

/**
 * @brief   Send a command to the TM1621 deriver IC.
 * @param   cmd : The sprecial command.
 * @return  None.
 */
void TM1621_SendCmd(const uint16_t cmd)
{
    uint32_t idx;           /* index.*/
    uint16_t cmd_frame;     /* Contain the command frame.*/
    uint16_t cur_frm_msk;   /* Current bit of frame mask.*/

    /** Create command frame for transmit.*/
    cmd_frame = cmd | TM1621_CMD_Msk;
    
    /* Transmit command.*/
    TM1621_SET_CS(Bit_RESET);   /* */
    for (idx = LENGTH_OF_FRAME_CMD; idx > 0; idx--)
    {
        TM1621_SET_WR(Bit_RESET);
        cur_frm_msk = 0x0001 << (idx - 1);
        TM1621_SET_DAT(cmd_frame & cur_frm_msk);
        // TM_DELAY();
        TM1621_SET_WR(Bit_SET);
        // TM_DELAY();
    }
    /* Set pin as idle status.*/
    TM1621_SET_IDLE();
    // TM_DELAY();
}

/**
 * @brief   Send a data to ram at the special address.
 */
void TM1621_SendSingleDat(const uint16_t ram_add, const uint16_t dat)
{
    uint32_t idx;           /* index*/
    uint16_t dat_frm;       /* Data frame.*/
    uint16_t cur_frm_msk;   /* Current bit of frame mask.*/

    /** Create command frame for transmit.*/
    dat_frm = ((ram_add << TM1621_RAM_ADD_Pos)&TM1621_RAM_ADD_Msk)|(dat & TM1621_RAM_DAT_Msk)|TM1621_DAT_Msk;

    /* Transmit command.*/
    TM1621_SET_CS(Bit_RESET);   /* */
    // TM_DELAY();
    for (idx = LENGTH_OF_FRAME_DAT; idx > 0; idx--)
    {
        TM1621_SET_WR(Bit_RESET);
        cur_frm_msk = 0x0001 << (idx - 1);
        TM1621_SET_DAT(dat_frm & cur_frm_msk);
        // TM_DELAY();
        TM1621_SET_WR(Bit_SET);
        // TM_DELAY();
    }

    /* Set pin as idle status.*/
    TM1621_SET_IDLE();
    // TM_DELAY();
}

/**
 * @brief   Send muntil data to TM1621's RAM.
 * @param   ram_add  Ram address.
 * @param   p_dat    Pointer to the data buffer containing the display data.
 * @param   num_data: Num of data.
 * @return  None
 */
void TM1621_SendMulDat(const uint16_t ram_add, uint8_t *p_dat, uint8_t num_data)
{
    uint32_t idx;           /* index*/
    uint16_t fst_frm;       /* Special first frame contain first address.*/
    uint16_t cur_frm_msk;   /* Current bit of frame mask.*/
    uint16_t idx_of_byte;   /* index of byte in buffer.*/
    uint16_t num_of_bit;

    fst_frm = ((ram_add << TM1621_RAM_ADD_Pos) & TM1621_RAM_ADD_Msk) | TM1621_DAT_Msk;
    num_of_bit = ((uint16_t) num_data) << 2;

    /** Send the first without data.*/
    TM1621_SET_CS(Bit_RESET);   /* */
    // TM_DELAY();
    for (idx = LENGTH_OF_FRAME_DAT; idx > 4; idx--)
    {
        TM1621_SET_WR(Bit_RESET);
        cur_frm_msk = 0x0001 << (idx - 1);
        TM1621_SET_DAT(fst_frm & cur_frm_msk);
        // TM_DELAY();
        TM1621_SET_WR(Bit_SET);
        // TM_DELAY();
    }
    
    /* Send the data.*/
    for (idx = 0; idx < num_of_bit; idx++)
    {
        idx_of_byte = idx >> 3;
        cur_frm_msk = 0x0001u << (idx & 0x07u);
        TM1621_SET_WR(Bit_RESET);
        TM1621_SET_DAT(p_dat[idx_of_byte] & cur_frm_msk);
        // TM_DELAY();
        TM1621_SET_WR(Bit_SET);
        // TM_DELAY();
    }
    
    /* Set pin as idle status.*/
    TM1621_SET_IDLE();
    // TM_DELAY();
}
/*******************************************************************************
 * End code.
*******************************************************************************/
