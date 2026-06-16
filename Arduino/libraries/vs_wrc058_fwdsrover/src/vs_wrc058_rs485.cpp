#include "vs_wrc058_rs485.h"
#include <iostream>
#include <vector>
#include <algorithm>    
#include <iterator>  

#include <Arduino.h>
#include <driver/uart.h>


HardwareSerialRS485::HardwareSerialRS485(HardwareSerial* io){
  iostream_ = io;
}

uint8_t HardwareSerialRS485::idToNum(uint8_t id){
  uint8_t i;
  for(i = 0; i < 8; i++){
    if(stmIdList[i] == id){
      return (uint8_t)pow(2,i);
    }
    //i++;
  }

  return 0;
}

void HardwareSerialRS485::setDEPin(int dePin){
  this->dePinNr = dePin;
  this->dePinSt = false;
  pinMode(dePin, OUTPUT);
  digitalWrite(dePin,false);

  stmIdList[0] = S_MAIN;
  stmIdList[1] = S_STEER;
  stmIdList[2] = S_NONE;
  stmIdList[3] = S_NONE;
  stmIdList[4] = S_NONE;
  stmIdList[5] = S_NONE;
  stmIdList[6] = S_NONE;
  stmIdList[7] = S_NONE;
  stmIdList[8] = S_BRC;



  //uart_enable_intr_mask(UART_NUM_2, UART_INT_ENA_REG || UART_TX_DONE_INT_ENA);


}


void HardwareSerialRS485::onDE(){
  digitalWrite(dePinNr, true);
  dePinSt = true;
}

void HardwareSerialRS485::offDE(){
  digitalWrite(dePinNr, false);
  dePinSt = false;
}

bool HardwareSerialRS485::beginTransmission(){

  //送信中の場合はリターン
  if(dePinSt == true){
      return false;
  }

  while(iostream_->available()){
    iostream_->read();
  }

  txTimeStart = micros();

  //DEピンをHIGHに
  this->onDE();

  return true;
}

void HardwareSerialRS485::endTransmission(){
    
  //送信終了まで待機
  iostream_->flush();

  //delayMicroseconds(30);

  while(micros() - txTimeStart < txTimeReq){
    delayMicroseconds(2);
  }

  //DEピンをLOWに
  this->offDE();

  return;
}

std::string rcvMsgViaRS485(256, '\0');
bool HardwareSerialRS485::getMsg(){

  //送信中ならリターン
  if(dePinSt){
      return false;
  }

  while(iostream_->available()){
    rcvMsgViaRS485.push_back(iostream_->read());
/*        
    //改行コードならメッセージ終了
    if(rcvMsgViaRS485[rcvMsgViaRS485.length()-1] == '\n'){
      return true;
    }
*/
    //バッファが256字ならクリア
    if((int)rcvMsgViaRS485.length() > 255){
      rcvMsgViaRS485.clear();
    }
  }

  return false;
}

//STMのメモリマップから指定の部分をreadする
int HardwareSerialRS485::readMemmap(struct stmCmd readCmd){
  uint8_t msg[MAX_MSG_SIZE];
  uint8_t csum = 0;
  uint8_t msgLength = 0;
  int16_t rtnMsgLength = 0;
  uint8_t dummy_msg_bytes = 2;

  msg[0] = 0x00;
  msgLength++;
  msg[1] = 0x00;
  msgLength++;

  //readコマンドセット
  msg[0+dummy_msg_bytes] = 'R';
  csum = 'R';
  msgLength++;

  //読み込みアドレスをセット
  (*(uint16_t *)(&msg[1+dummy_msg_bytes])) = readCmd.addr;
  csum += readCmd.addr;
  msgLength += 2;

  //読み込みアドレスが0xff以下だった場合、上位バイトをクリア
  if(readCmd.addr <= 0xff){
    msg[2+dummy_msg_bytes] = 0x00;
  }

  //読み込む長さをセット
  msg[3+dummy_msg_bytes] = readCmd.length;
  csum += readCmd.length;
  msgLength++;

  //読み込むデバイス数とID,チェックサムをセット
  int i = 0;
  int devCnt = 0;
  if(readCmd.id == 0xff){ //ブロードキャスト
    devCnt = NUM_STM;
    msg[4+dummy_msg_bytes] = NUM_STM;
    csum  += NUM_STM; 
    msgLength++;

    msg[5+dummy_msg_bytes] = S_BRC;
    csum  += S_BRC;
    msgLength++;

    //チェックサムセット
    msg[6+dummy_msg_bytes] = 0x100 - csum;
    msgLength++;

  }else{
    for(i = 0; i < 8; i++){
      if(readCmd.id & (0x01 << i)){
        msg[5 + devCnt+dummy_msg_bytes] = stmIdList[i];
        devCnt++;
        csum += stmIdList[i];
        msgLength++;
      } 
    }

    msg[4+dummy_msg_bytes] = (uint8_t)devCnt;
    csum  += devCnt;
    msgLength++;


    //チェックサムセット
    msg[5 + devCnt+dummy_msg_bytes] = 0x100 - csum;
    msgLength++;

  }

  //STMからのリターンメッセージの長さの計算
  rtnMsgLength = devCnt * (readCmd.length + 2);
/*
  for(i = 0; i < msgLength; i++){
    Serial.print(msg[i],HEX);
    Serial.print(" ");
  }
  Serial.println();
*/

  //送信時間計算
  txTimeReq = msgLength*22;

  //読み込みメッセージ送信
  while(this->beginTransmission() == false){
    delayMicroseconds(100);
  }
  //for(i = 0; i < msgLength; i++){
  //  iostream_->write((char)msg[i]);
  //}
  iostream_->write(msg, msgLength);
  this->endTransmission();

  //クリア
  rcvMsgViaRS485.clear();

  int32_t getMsgStartTime = micros();
  while((int)rcvMsgViaRS485.length() < rtnMsgLength/*+msgLength*/){
    this->getMsg();

    //2000us以上経ってもリターンメッセージが揃わない場合は、通信エラーとする
    if(micros()-getMsgStartTime > 10000){
      
      Serial.print("485err_T: ");
      Serial.print(msg[5+dummy_msg_bytes], HEX);
      Serial.print(" ");
      Serial.println(micros()-getMsgStartTime);
      for(i = 0; i < msgLength; i++){
        Serial.print(msg[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      for(i = 0; i < (int)rcvMsgViaRS485.length(); i++){
        Serial.print((int)rcvMsgViaRS485[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      
      rcvMsgViaRS485.clear();
      rtnMsgLength = -1;
      //break;
      return (int)rtnMsgLength;
    }

    if((int)rcvMsgViaRS485.length() > rtnMsgLength/*+msgLength*/){
      Serial.print("485err_M: ");
      Serial.print(msg[5+dummy_msg_bytes], HEX);
      Serial.print(" ");
      Serial.println((int)rcvMsgViaRS485.length());
      for(i = 0; i < msgLength; i++){
        Serial.print(msg[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      for(i = 0; i < (int)rcvMsgViaRS485.length(); i++){
        Serial.print((int)rcvMsgViaRS485[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      rcvMsgViaRS485.clear();
      rtnMsgLength = -1;
      return (int)rtnMsgLength;
    }
  
  }  

  //rcvMsgViaRS485.erase(0,msgLength);

  //rcvMsgViaRS485.clear();

  //受信データのチェックサム確認
  uint8_t rChkSum = 0;
  for(i = 0; i < rtnMsgLength; i++){
    rChkSum += (uint8_t)rcvMsgViaRS485[i];
  }
  //Serial.println(rChkSum);
  if(rChkSum){
    Serial.println("485err_S");
    rcvMsgViaRS485.clear();
    rtnMsgLength = -1;
  }
/*
  Serial.print("485_OK: ");
  Serial.println(msg[5], HEX);
  for(i = 0; i < msgLength; i++){
    Serial.print(msg[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
*/
/*
  for(i = 0; i < rtnMsgLength; i++){
    Serial.print((int)rcvMsgViaRS485[i]);
    Serial.print(" ");
  }
  Serial.println();
*/
  
  return (int)rtnMsgLength;



}


//STMのメモリマップの指定の部分にwriteする
int HardwareSerialRS485::writeMemmap(struct stmCmd writeCmd){
  uint8_t msg[MAX_MSG_SIZE];
  uint8_t csum = 0;
  uint8_t msgLength = 0;
  int16_t rtnMsgLength = 0;
  uint8_t dummy_msg_bytes = 2;

  msg[0] = 0x00;
  msgLength++;
  msg[1] = 0x00;
  msgLength++;

  //writeコマンドセット
  msg[0+dummy_msg_bytes] = writeCmd.cmdType;
  csum = writeCmd.cmdType;
  msgLength++;

  //書込みアドレスをセット
  (*(uint16_t *)(&msg[1+dummy_msg_bytes])) = writeCmd.addr;
  csum += writeCmd.addr;
  msgLength += 2;

  //書込みアドレスが0xff以下だった場合、上位バイトをクリア
  if(writeCmd.addr <= 0xff){
    msg[2+dummy_msg_bytes] = 0x00;
  }

  //書き込む長さをセット
  msg[3+dummy_msg_bytes] = writeCmd.length;
  csum += writeCmd.length;
  msgLength++;

  //書き込むデバイス数とIDをセット
  int i = 0;
  int devCnt = 0;
  if(writeCmd.id == 0xff){ //ブロードキャスト
    devCnt = 1;
    msg[4+dummy_msg_bytes] = NUM_STM;
    csum  += NUM_STM; 
    msgLength++;

    msg[5+dummy_msg_bytes] = S_BRC;
    csum  += S_BRC;
    msgLength++;

  }else{
    for(i = 0; i < 8; i++){
      if(writeCmd.id & (0x01 << i)){
        msg[5 + devCnt+dummy_msg_bytes] = stmIdList[i];
        devCnt++;
        csum += stmIdList[i];
        msgLength++;
      } 
    }

    msg[4+dummy_msg_bytes] = (uint8_t)devCnt;
    csum  += devCnt;
    msgLength++;

  }

  //書き込むデータをセット
  for(i = 0; i < writeCmd.length; i ++){
    msg[msgLength] = writeCmd.value[i];
    msgLength++;
    csum += writeCmd.value[i];
  }

  //チェックサムセット
  msg[msgLength] = 0x100 - csum;
  msgLength++;
/*
  for(i = 0; i < msgLength; i++){
    //printf("%c",msg[i]);
    Serial.print(msg[i],HEX);
    Serial.print(" ");
  }
  Serial.println();
*/

  //送信時間計算
  txTimeReq = msgLength*22;

  //書き込みメッセージ送信
  while(this->beginTransmission() == false){
    delayMicroseconds(100);
  }
  for(i = 0; i < msgLength; i++){
    iostream_->write((char)msg[i]);
  }
  this->endTransmission();

  if(writeCmd.cmdType == 'w'){
    return 1;
  }

  //クリア
  rcvMsgViaRS485.clear();

  int32_t getMsgStartTime = micros();
  while((int)rcvMsgViaRS485.length() < 1){
    this->getMsg();

    //2000us以上経ってもリターンメッセージが揃わない場合は、通信エラーとする
    if(micros()-getMsgStartTime > 2000){
      rcvMsgViaRS485.clear();
      return 0;
    }
  
  }

  if(rcvMsgViaRS485[0] == 0x01){
    //書き込み成功
    rcvMsgViaRS485.clear();
    return 1;
  }else{
    //書き込み失敗
    rcvMsgViaRS485.clear();
    return 0;
  }

  return 0;
}




//HardwareSerialRS485 SerialSTM(&Serial2);


