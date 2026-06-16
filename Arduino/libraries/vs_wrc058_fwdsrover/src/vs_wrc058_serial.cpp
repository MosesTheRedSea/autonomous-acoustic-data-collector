#include "vs_wrc058_serial.h"
#include "vs_wrc058_memmap.h"
#include <iostream>
#include <vector>
#include <algorithm>    
#include <iterator> 


//Serialで受信したメッセージを2次バッファにため、コマンドとして成立し次第
//メモリマップread/writeまたはROSに流す
std::string rcvMsgViaSerial(SERIAL_BUF_SIZE, '\0');
std::string rcvMsg4ROS(SERIAL_BUF_SIZE, '\0');
int rosMsgTop = 0;
int rosMsgBottom = 0;
int ringMask = SERIAL_BUF_SIZE -1;
int numChars = 0;
const uint32_t serialWaitTimeoutTime = 200;
int persMsgViaSerial(){
  static uint32_t serialWaitingTime = millis();
  int return_val = 0;

  static int16_t rosMsgBytes = 0; //ROSコマンドのbyte数　msg_length+8byte
  char tmp;

  //while(Serial.available()){

  //バッファ空＝新規コマンドなら先頭文字を取得
  if(rcvMsgViaSerial.empty()){
    tmp = Serial.read();
    rcvMsgViaSerial.push_back(tmp);
    serialWaitingTime = millis();
  }

  

  //先頭文字r/R/w/Wならメモリマップ操作，ffならROS．それ以外は破棄．
  if(rcvMsgViaSerial[0] == 'r' || rcvMsgViaSerial[0] == 'R' || rcvMsgViaSerial[0] == 'w' || rcvMsgViaSerial[0] == 'W'){
    //メモリマップ操作
    while(Serial.available()){
      //serialWaitingTime = millis();

      tmp = Serial.read();

      if(tmp == 'r' || tmp == 'R' || tmp == 'w' || tmp == 'W'){
        rcvMsgViaSerial.clear();
      }

      rcvMsgViaSerial.push_back(tmp);

      if(rcvMsgViaSerial.length() >= SERIAL_BUF_SIZE){
        rcvMsgViaSerial.clear();
      }

      //改行コード取得でコマンド成立
      if(tmp == '\n'){
        //Serial.print(rcvMsgViaSerial.c_str());
        return_val += checkI2cAddrOfMsg(rcvMsgViaSerial.c_str(), rcvMsgViaSerial.length()-1);
        
        rcvMsgViaSerial.clear();
        serialWaitingTime = millis();
        break;
      }
    }

    if(millis() - serialWaitingTime > serialWaitTimeoutTime){
      rcvMsgViaSerial.clear();
    }

    

  }else if(rcvMsgViaSerial[0] == 0xff){
    //ROSコマンド

    //rosMsgBytes == 0なら、4byte目までを取得して全体のbyte数を算出
    while(rosMsgBytes==0){
      if(millis() - serialWaitingTime > serialWaitTimeoutTime){
        serialWaitingTime = millis();
        rcvMsgViaSerial.clear();
        rosMsgBytes = 0;
        return return_val;
      }
      if(!Serial.available()){
        return return_val;
      }
      
      tmp = Serial.read();
      rcvMsgViaSerial.push_back(tmp);

      if(rcvMsgViaSerial.length() >= 4){
        rosMsgBytes = (int)rcvMsgViaSerial[2];
        rosMsgBytes += (int)rcvMsgViaSerial[3] << 8;
        rosMsgBytes += 8;

        if(rosMsgBytes > SERIAL_BUF_SIZE){
          rosMsgBytes = SERIAL_BUF_SIZE;
        }

        //Serial.println(rosMsgBytes);
      }
    }
    //serialWaitingTime = millis();

    //bytes取得済みなら、指定のメッセージ長になるまでread
    while(rcvMsgViaSerial.length() < rosMsgBytes){
      if(millis() - serialWaitingTime > serialWaitTimeoutTime){
        serialWaitingTime = millis();
        rcvMsgViaSerial.clear();
        rosMsgBytes = 0;
        return return_val;
      }

      if(!Serial.available()){
        return return_val;
      }
      //serialWaitingTime = millis();

      tmp = Serial.read();
      rcvMsgViaSerial.push_back(tmp);

    }


    while(rosMsgBottom >= SERIAL_BUF_SIZE){
      rosMsgBottom -= SERIAL_BUF_SIZE;
    }
    while(rosMsgTop >= SERIAL_BUF_SIZE){
      rosMsgTop -= SERIAL_BUF_SIZE;
    }

    //3次バッファが一杯なら、上書きしてbottomを進める
    if(numChars + rosMsgBytes > SERIAL_BUF_SIZE){
      for(int i = 0; i < rosMsgBytes; i++){
        rcvMsg4ROS[(rosMsgTop + i) & ringMask] = rcvMsgViaSerial[i];
      }
      numChars = SERIAL_BUF_SIZE;
      rosMsgTop += rosMsgBytes;
      rosMsgBottom = (rosMsgTop & ringMask) +1;
      rcvMsgViaSerial.clear();
      rosMsgBytes = 0;
    }else{
      for(int i = 0; i < rosMsgBytes; i++){
        rcvMsg4ROS[(rosMsgTop + i) & ringMask] = rcvMsgViaSerial[i];
      }
      numChars += rosMsgBytes;
      rosMsgTop += rosMsgBytes;
      rcvMsgViaSerial.clear();
      rosMsgBytes = 0;
    }

    serialWaitingTime = millis();
    return return_val;
    

  }else{
    rcvMsgViaSerial.clear();
  }


  //}

  return return_val;

}

//下方互換
int readMsgViaSerial(){
  return persMsgViaSerial();
}

//ROS用3次バッファのreadコマンド
int readMsg4ROS(){

  while(rosMsgBottom >= SERIAL_BUF_SIZE){
    rosMsgBottom -= SERIAL_BUF_SIZE;
  }
  while(rosMsgTop >= SERIAL_BUF_SIZE){
    rosMsgTop -= SERIAL_BUF_SIZE;
  }

  if(numChars <= 0){
    if(Serial.available()){
      persMsgViaSerial();
    }
    if(numChars <= 0){
      return -1;
    }
  }

  int return_val = rcvMsg4ROS[rosMsgBottom];
  rosMsgBottom++;
  numChars--;
  

  return return_val;


}

//パースバッファのクリア
void clearPersBuf(){
  rcvMsgViaSerial.clear();
  rcvMsg4ROS.clear();
}


