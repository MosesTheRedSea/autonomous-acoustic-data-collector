#include <Wire.h>
#include "vs_wrc058_i2c.h"
#include "vs_wrc058_memmap.h"

//I2Cをマスターモードで起動
void i2cMasterInit(){
  pinMode(21, INPUT);
  pinMode(22, INPUT);

  uint32_t frequency=100000;
  Wire.begin(21, 22, frequency);

  delay(3);

  return;
}

//I2Cをスレーブモードで起動
void i2cSlaveInit(uint8_t addr){
  pinMode(21, INPUT);
  pinMode(22, INPUT);

  Wire.begin(addr);

  delay(3);

  return;
}

