/**
 * VS-WRC051上のESP32とSTM32間のi2c通信に関するライブラリ
 */
#ifndef WRC051_I2C_H
#define WRC051_I2C_H

#include <Arduino.h>
#include <Wire.h>
#include "vs_wrc058_memmap.h"

void i2cMasterInit();
void i2cSlaveInit(uint8_t addr);





#endif /* I2C_H */
