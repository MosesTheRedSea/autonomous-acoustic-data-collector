/**
 * オリエンタルモータ用RS485通信に関するライブラリ
 * 
 * 通信設定（設定は本体のスイッチとMEXE02で変更）
 * 通信速度：115200
 * パリティ：なし
 * ストップビット：1ビット
 * 送信待ち時間：2ms
 * 通信タイムアウト：監視無し
 * 通信異常アラーム：3回
 * 
 * ID：
 * シリンダー：1(0x01)
 * 
 */
#ifndef WRC058_OM_RS485_H
#define WRC058_OM_RS485_H

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
#define NUM_OM 1    //ローバーのオリエンタルモータの数
#define NUM_LIST 0
#define ID_LIST 1

struct omrwCmd{
  uint8_t cmdType;  //(読込)R, （書込）W or w
  uint8_t id;       //読み書きする基板のid(テーブル参照)
  uint16_t addr;    //読み書きするメモリマップの先頭アドレス
  uint8_t *value;   //書き込む値
  uint8_t length;   //読み書きする値の長さ
};

enum omId{
  O_NONE = 0x00,   //なし
  O_CYLINDER = 0x10,   //VS-WRC051上のSTM
  O_BRC  = 0xff    //ブロードキャスト
};


extern std::string rcvMsgViaOMRS485;

class HardwareSerialOMRS485;

class HardwareSerialOMRS485 : public HardwareSerial{
public:
  uint8_t omIdList[9];

  HardwareSerialOMRS485(int uart_nr);

  uint8_t idToNum(uint8_t id);
  void setDEPin(int dePin=13);    //DEピンをセット、DE制御用のマルチタスクをセットアップする
  bool chkDE();
  void onDE();                    //DEピンをオンにする
  void offDE();                   //DEピンをオフにする
  bool beginTransmission();       //通信を始める
  void endTransmission();         //通信を終わる
  bool getMsg();                  //メッセージを受信する
  int  readMemmap(struct omrwCmd readCmd);              //STMのメモリマップを読み込む
  int  writeMemmap(struct omrwCmd writeCmd);            //STMのメモリマップに書き込む


protected:
  int dePinNr = 13;
  bool dePinSt = false;
  


};

uint16_t calcCRC16(uint8_t *msg, uint8_t msg_length);  //CRC-16を計算する

extern HardwareSerialOMRS485 SerialOM;

#endif /* OM_RS485_H */