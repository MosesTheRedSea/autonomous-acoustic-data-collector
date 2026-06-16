/**
 * 基板間を繋ぐRS485通信に関するライブラリ
 */
#ifndef WRC051_RS485_H
#define WRC051_RS485_H

//#include <pins_arduino.h>
//#include <Arduino.h>
#include <HardwareSerial.h>
//#include <Stream.h>
//#include <inttypes.h>
#include <iostream>
#include <vector>
#include <algorithm>    
#include <iterator>  
/*
#include <rom/ets_sys.h>
#include <esp_attr.h>
#include <esp_intr.h>
#include <rom/uart.h>
#include <soc/uart_reg.h>
#include <soc/uart_struct.h>
#include <soc/io_mux_reg.h>
#include <soc/gpio_sig_map.h>
#include <soc/dport_reg.h>
#include <soc/rtc.h>
#include <esp_intr_alloc.h>
*/

#define MAX_MSG_SIZE 0x80
#define NUM_STM 2    //ローバーのSTMの数
#define NUM_LIST 0
#define ID_LIST 1

struct stmCmd{
  uint8_t cmdType;  //(読込)R, （書込）W or w
  uint8_t id;       //読み書きする基板のid(テーブル参照)
  uint16_t addr;    //読み書きするメモリマップの先頭アドレス
  uint8_t *value;   //書き込む値
  uint8_t length;   //読み書きする値の長さ
};

enum stmId{
  S_NONE = 0x00,   //なし
  S_MAIN = 0x10,   //VS-WRC051上のSTM
  S_STEER = 0x1f,   //VS-WRC053上のSTM
  S_BRC  = 0xff    //ブロードキャスト
};


extern std::string rcvMsgViaRS485;

class HardwareSerialRS485{
public:
  uint8_t stmIdList[9];

  HardwareSerialRS485(HardwareSerial* io);

  uint8_t idToNum(uint8_t id);
  void setDEPin(int dePin=13);    //DEピンをセット、DE制御用のマルチタスクをセットアップする
  void onDE();                    //DEピンをオンにする
  void offDE();                   //DEピンをオフにする
  bool beginTransmission();       //通信を始める
  void endTransmission();         //通信を終わる
  bool getMsg();                  //メッセージを受信する
  int  readMemmap(struct stmCmd readCmd);              //STMのメモリマップを読み込む
  int  writeMemmap(struct stmCmd writeCmd);            //STMのメモリマップに書き込む


protected:
  int dePinNr = 13;
  bool dePinSt = false;
  int32_t txTimeReq = 0;
  int32_t txTimeStart = 0;
  HardwareSerial* iostream_;
  


};



//extern HardwareSerialRS485 SerialSTM;

#endif /* RS485_H */