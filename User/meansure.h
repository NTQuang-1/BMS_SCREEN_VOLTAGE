
#ifndef MEANSURE_H
#define MEANSURE_H


#include "ch32v00x.h"


#define MEANSURE_SRC_Internal   0x0000
#define MEANSURE_SRC_External   0x0008


uint32_t MeansureInit(void);
void MeansureCalibBatVolt(uint16_t calib_vref, int16_t calib_offset, float calib_gain, float mul);
void MeansureAutoCalid(void);
int MeansureBatVolt(void);
int16_t MeansureTempe(void);
#endif /*MEANSURE_H*/