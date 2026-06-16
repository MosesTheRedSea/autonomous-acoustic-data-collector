#include "vs_wrc058_om_rs485.h"
#include <iostream>
#include <vector>
#include <algorithm>    
#include <iterator>  

#include <Arduino.h>
#include <driver/uart.h>


HardwareSerialOMRS485::HardwareSerialOMRS485(int uart_nr) : HardwareSerial(uart_nr) {}

uint8_t HardwareSerialOMRS485::idToNum(uint8_t id){
  uint8_t i;
  for(i = 0; i < 8; i++){
    if(omIdList[i] == id){
      return (uint8_t)pow(2,i);
    }
  }

  return 0;
}

void HardwareSerialOMRS485::setDEPin(int dePin){
  dePinNr = dePin;
  dePinSt = false;
  pinMode(dePin, OUTPUT);
  digitalWrite(dePin,false);

  omIdList[0] = O_CYLINDER;
  omIdList[1] = O_NONE;
  omIdList[2] = O_NONE;
  omIdList[3] = O_NONE;
  omIdList[4] = O_NONE;
  omIdList[5] = O_NONE;
  omIdList[6] = O_NONE;
  omIdList[7] = O_NONE;
  omIdList[8] = O_BRC;



  //uart_enable_intr_mask(UART_NUM_2, UART_INT_ENA_REG || UART_TX_DONE_INT_ENA);


}

bool chkDE(){

}

void HardwareSerialOMRS485::onDE(){
  digitalWrite(dePinNr, true);
  dePinSt = true;
}

void HardwareSerialOMRS485::offDE(){
  digitalWrite(dePinNr, false);
  dePinSt = false;
}

bool HardwareSerialOMRS485::beginTransmission(){

  //送信中の場合はリターン
  if(dePinSt == true){
      return false;
  }

  //DEピンをHIGHに
  onDE();

  return true;
}

void HardwareSerialOMRS485::endTransmission(){
    
  //送信終了まで待機
  flush();

  while(SerialOM.available()){
    SerialOM.read();
  }

  //DEピンをLOWに
  offDE();

  return;
}

std::string rcvMsgViaOMRS485(256, '\0');
bool HardwareSerialOMRS485::getMsg(){

  //送信中ならリターン
  if(dePinSt){
      return false;
  }

  while(available()){
    rcvMsgViaOMRS485.push_back(read());
/*        
    //改行コードならメッセージ終了
    if(rcvMsgViaOMRS485[rcvMsgViaOMRS485.length()-1] == '\n'){
      return true;
    }
*/
    //バッファが256字ならクリア
    if((int)rcvMsgViaOMRS485.length() > 511){
      rcvMsgViaOMRS485.clear();
    }
  }

  return true;
}

//STMのメモリマップから指定の部分をreadする
int HardwareSerialOMRS485::readMemmap(struct omrwCmd readCmd){
  uint8_t msg[MAX_MSG_SIZE];
  uint8_t csum = 0;
  uint8_t msgLength = 0;
  int16_t rtnMsgLength = 0;
  int i;

  //スレーブアドレスセット
  for(i = 0; i < 8; i++){
    if(readCmd.id == (uint8_t)pow(2,i)){
      msg[0] = omIdList[i];
    } 
  }
  msgLength++;

  //readコマンドセット
  msg[1] = 0x03;
  msgLength++;

  //読み込みアドレスをセット
  msg[2] = readCmd.addr>>8;
  msg[3] = readCmd.addr;
  msgLength += 2;

  //読み込む長さをセット
  msg[4] = 0x00;
  msg[5] = readCmd.length;
  msgLength += 2;

  //crcをセット
  (*(uint16_t *)(&msg[6])) = calcCRC16(msg, msgLength);
  msgLength += 2;


  //STMからのリターンメッセージの長さの計算
  rtnMsgLength = readCmd.length*2 +5;

  //読み込みメッセージ送信
  while(beginTransmission() == false){
    delayMicroseconds(100);
  }
  for(i = 0; i < msgLength; i++){
    write((char)msg[i]);
  }
  endTransmission();

  //クリア
  rcvMsgViaOMRS485.clear();

  int32_t getMsgStartTime = micros();
  while((int)rcvMsgViaOMRS485.length() < rtnMsgLength){
    getMsg();

    //2000us以上経ってもリターンメッセージが揃わない場合は、通信エラーとする
    if(micros()-getMsgStartTime > 200000){
      Serial.print("485err_T: ");
      Serial.println(msg[0], HEX);
      rcvMsgViaOMRS485.clear();
      rtnMsgLength = -1;
      //break;
      return (int)rtnMsgLength;
    }

    if((int)rcvMsgViaOMRS485.length() > rtnMsgLength){
      Serial.print("485err_M: ");
      Serial.println(msg[0], HEX);
      for(i = 0; i < msgLength; i++){
        Serial.print(msg[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      for(i = 0; i < rcvMsgViaOMRS485.length(); i++){
        Serial.print((int)rcvMsgViaOMRS485[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      rcvMsgViaOMRS485.clear();
      rtnMsgLength = -1;
      return (int)rtnMsgLength;
    }
  
  }  

  //受信データのCRC確認
  uint16_t rcvCrc = calcCRC16((uint8_t*)(&rcvMsgViaOMRS485[0]), rtnMsgLength-2);
  if(rcvCrc != (*(uint16_t*)(&rcvMsgViaOMRS485[rtnMsgLength-2]))){
    Serial.println("485err_C");
    rcvMsgViaOMRS485.clear();
    rtnMsgLength = -1;
  }
  
  
  return (int)rtnMsgLength;



}


//STMのメモリマップの指定の部分にwriteする
int HardwareSerialOMRS485::writeMemmap(struct omrwCmd writeCmd){
  uint8_t msg[MAX_MSG_SIZE];
  uint8_t csum = 0;
  uint8_t msgLength = 0;
  int16_t rtnMsgLength = 0;
  int i;

  //スレーブアドレスセット
  for(i = 0; i < 8; i++){
    if(writeCmd.id == (uint8_t)pow(2,i)){
      msg[0] = omIdList[i];
    } 
  }
  msgLength++;

  //writeコマンドセット
  msg[1] = 0x10;
  msgLength++;

  //書込みアドレスをセット
  msg[2] = writeCmd.addr>>8;
  msg[3] = writeCmd.addr;
  msgLength += 2;

  //書き込むレジスタ数をセット
  msg[4] = 0x00;
  msg[5] = writeCmd.length;
  msgLength += 2;

  //書込みバイト数
  msg[6] = writeCmd.length*2;
  msgLength++;

  //書込みデータをセット
  for(i = 0; i < writeCmd.length*2; i++){
    msg[msgLength] = writeCmd.value[i];
    msgLength++;
  }

  //crcをセット
  (*(uint16_t *)(&msg[msgLength])) = calcCRC16(msg, msgLength);
  msgLength += 2;


  //書き込みメッセージ送信
  while(beginTransmission() == false){
    delay(1);
  }
  for(i = 0; i < msgLength; i++){
    write((char)msg[i]);
  }
  endTransmission();

  //クリア
  rcvMsgViaOMRS485.clear();

  int32_t getMsgStartTime = micros();
  while((int)rcvMsgViaOMRS485.length() < 1){
    getMsg();

    //2000us以上経ってもリターンメッセージが揃わない場合は、通信エラーとする
    if(micros()-getMsgStartTime > 200000){
      rcvMsgViaOMRS485.clear();
      return 0;
    }
  
  }

  if(rcvMsgViaOMRS485[0] == msg[0]){
    //書き込み成功
    rcvMsgViaOMRS485.clear();
    return 1;
  }else{
    //書き込み失敗
    rcvMsgViaOMRS485.clear();
    return 0;
  }

  return 0;
}




HardwareSerialOMRS485 SerialOM(1);

uint16_t calcCRC16(uint8_t *msg, uint8_t msgLength){

  uint16_t crc_register = 0xFFFF;
  int i, j;
  for(i = 0; i < msgLength; i++){
    crc_register ^= msg[i];
    for(j = 0; j < 8; j++){
      if(crc_register & 0x0001){
        crc_register = crc_register >> 1;
        crc_register ^= 0xA001;
      }else{
        crc_register = crc_register >> 1;
      }
    }
  }

  return crc_register;
}