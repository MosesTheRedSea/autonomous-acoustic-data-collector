#include "vs_wrc058_spi.h"
#include <Arduino.h>
#include <SPI.h>

const uint8_t CMD_config_mode_enter[] =     {0x01,0x43,0x00,0x01, 0x00,0x00,0x00,0x00, 0x00};
const uint8_t CMD_config_mode_exit[] =      {0x01,0x43,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00};
const uint8_t CMD_config_mode_exit2[] =     {0x01,0x43,0x00,0x00, 0x5a,0x5a,0x5a,0x5a, 0x5a};
const uint8_t CMD_set_mode_and_lock[] =     {0x01,0x44,0x00,0x01, 0x03,0x00,0x00,0x00, 0x00};
const uint8_t CMD_query_model_and_mode[] =  {0x01,0x45,0x00,0x5a, 0x5a,0x5a,0x5a,0x5a, 0x5a};
const uint8_t CMD_vibration_enable[] =      {0x01,0x4d,0x00,0x00, 0x01,0xff,0xff,0xff, 0xff};
const uint8_t CMD_vibration_disnable[] =    {0x01,0x4d,0x00,0xff, 0xff,0xff,0xff,0xff, 0xff};
const uint8_t CMD_query_DS2_analog_mode[] = {0x01,0x41,0x00,0x5a, 0x5a,0x5a,0x5a,0x5a, 0x5a};
const uint8_t CMD_set_DS2_native_mode[] =   {0x01,0x4f,0x00,0xff, 0xff,0x03,0x00,0x00, 0x00};
const uint8_t CMD_read_data[] =             {0x01,0x42,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00};
const uint8_t CMD_read_data2[] =            {0x01,0x42,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
                                             0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00};


uint8_t pad_buf[30];      //PAD入力データ用バッファ
uint8_t prev_pad_buf[30]; //前回取得したPAD入力データ

int pad_time = 0;     //最後にPAD入力を取得した時刻


Pad pad_data;
Pad prev_pad_data;
spi_t* spi_pad;

//SPI初期設定（VS-C3用）
void spiInit(){

  spi_pad = spiStartBus(VSPI, 2500000, SPI_MODE2, SPI_LSBFIRST);

  pinMode(19, INPUT);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
 
  spiAttachSCK(spi_pad, 18);
  spiAttachMISO(spi_pad, 19);
  spiAttachMOSI(spi_pad, 23);

  int i = 0;
  for(i = 0; i < sizeof(pad_buf); i++){
    pad_buf[i] = 0x00;
  }
  
  checkAN();
}

//VS-C3に対してRead&Write
void rwPad(uint8_t* sendData, uint8_t* rcvData, uint16_t dataLength){
  int i = 0;
  //static uint32_t time1;
  //time1 = micros();
  digitalWrite(5, LOW);
  for(i = 0; i < dataLength; i++){
    spiTransferBytes(spi_pad, (uint8_t* )&sendData[i], (uint8_t* )&rcvData[i], 1);
    delayMicroseconds(10);
  }
  digitalWrite(5, HIGH);
  //Serial.println(micros()-time1);


}

//PADの入力情報を更新
int updatePad(){

  if(millis() - pad_time < 15){
    return 0;
  }
  pad_time = millis();

  rwPad((uint8_t*)CMD_read_data, (uint8_t*)pad_buf, (uint16_t)sizeof(CMD_read_data));

  //アナログモードでない場合はアナログモードに変更してreturn
  if(checkAN()){
    return 0;
  }

  

  //前回取得時とコントローラの状態が変化していない場合はリターン
  if(memcmp(pad_buf, prev_pad_buf, sizeof(CMD_read_data)) == 0){
    return 0;
  }

  int i;
  for(i = 0; i<sizeof(CMD_read_data); i++){
    prev_pad_buf[i] = pad_buf[i];
  }

  
  getButton();
  getAnalogStick();

  return 1;
}

//PADがアナログモードかどうかを判定し、アナログでなければアナログに書き換える
int checkAN(){

  if(pad_buf[ANF_ADDR] != 0x73){
    //デジタルモード

    //Serial.println("setANALOG");
    //delay(15);
    //コンフィギュレーションモードにenter
    rwPad((uint8_t*)CMD_config_mode_enter, (uint8_t*)pad_buf, (uint16_t)sizeof(CMD_config_mode_enter));

    delay(1);
    //パッドをアナログモードに設定
    rwPad((uint8_t*)CMD_set_mode_and_lock, (uint8_t*)pad_buf, (uint16_t)sizeof(CMD_set_mode_and_lock));

    delay(1);
    //コンフィギュレーションモードからexit
    rwPad((uint8_t*)CMD_config_mode_exit, (uint8_t*)pad_buf, (uint16_t)sizeof(CMD_config_mode_exit));
    //delay(15);



    return 1;
  }

  return 0;
}

//ボタンの入力情報を取得
void getButton(){
  pad_data.button = ~((pad_buf[BTN_ADDR] << 8) | pad_buf[BTN_ADDR +1]);
}

//アナログスティックの入力値をint8_tで取得
void getAnalogStick(){
  int8_t tmp[4];
  int i;
  
  for(i = 0; i < 4; i++){
    tmp[i] = (int8_t)pad_buf[ANS_ADDR+i];
    if((uint8_t)tmp[i] <= 0x7F){
      tmp[i] = 127 - tmp[i];
    }else{
      tmp[i] = -(128 + tmp[i]);
    }
  }

  pad_data.right_stick.x = tmp[0];
  pad_data.right_stick.y = tmp[1];
  pad_data.left_stick.x  = tmp[2];
  pad_data.left_stick.y  = tmp[3];

}

//ボタンの押下を確認 押されている->TRUE 押されていない->FALSE
uint8_t checkBTN(uint16_t comparisonData){
  
  if(pad_data.button & comparisonData){
    return 1;
  }else{
    return 0;
  } 
}


//VS-C3からの入力生データをSerialで送信（シリアルモニタ表示用）
void VS_C2ToSerial(){
  int i = 0;
  
  for(i = 0; i < 30; i++){
    Serial.print(pad_buf[i],HEX);
    Serial.print(' ');  
  }
  Serial.println();
}

//アナログスティックまたはいずれかのボタンに入力があるかどうか
bool existsPadInput(){
  if(pad_data.button){
    return true;
  }
  if(abs(pad_data.right_stick.x) > 10 || abs(pad_data.right_stick.y) > 10 || abs(pad_data.left_stick.x) > 10 || abs(pad_data.left_stick.y) > 10){
    return true;
  }

  return false;
  
}



