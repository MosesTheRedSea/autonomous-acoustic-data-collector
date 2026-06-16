/**
 * vs-wrc021、ESP32のVSPIライブラリ
 * VS-C3の接続に対応
 * DS2の規格に沿って通信を行う
 */
#ifndef WRC058_SPI_H
#define WRC058_SPI_H

#include <Arduino.h>
#include <SPI.h>

extern const uint8_t CMD_config_mode_enter[];
extern const uint8_t CMD_config_mode_exit[];
extern const uint8_t CMD_config_mode_exit2[];
extern const uint8_t CMD_set_mode_and_lock[];
extern const uint8_t CMD_query_model_and_mode[];
extern const uint8_t CMD_vibration_enable[];
extern const uint8_t CMD_vibration_disnable[];
extern const uint8_t CMD_query_DS2_analog_mode[];
extern const uint8_t CMD_set_DS2_native_mode[];
extern const uint8_t CMD_read_data[];
extern const uint8_t CMD_read_data2[];

#define ANF_ADDR   1
#define BTN_ADDR   3
#define ANS_ADDR   5

//アナログスティックのデータを保持
struct AStick{
  int8_t x;   //X方向の入力
  int8_t y;   //Y方向の入力
};

//VS-C3の入力データを保持
struct Pad{
  uint16_t button;  //ボタン入力
  AStick  right_stick;  //右アナログスティック
  AStick  left_stick;   //左アナログスティック
};

/**
 * VS-C3のボタンのリストです。
 * CROSS  -> 十字ボタン
 * START  -> スタートボタン
 * ANBTN  -> アナログスティック押し込み
 * SELECT -> セレクトボタン
 * SQUARE -> 四角ボタン
 * CROSS  -> バツボタン
 * CIRCLE -> 丸ボタン
 * TRIANGLE -> 三角ボタン
 * S_     -> ショルダーボタン
 */
enum padBTNList{
  CROSS_L = 0x8000,
  CROSS_D = 0x4000,
  CROSS_R = 0x2000,
  CROSS_U = 0x1000,
  BTN_START = 0x0800,
  ANBTN_R = 0x0400,
  ANBTN_L = 0x0200,
  BTN_SELECT = 0x0100,
  SQUARE = 0x0080,
  CROSS  = 0x0040,
  CIRCLE = 0x0020,
  TRIANGLE = 0x0010,
  S_R1   = 0x0008,
  S_L1   = 0x0004,
  S_R2   = 0x0002,
  S_L2   = 0x0001
};

extern uint8_t padBuf[];
extern uint8_t prev_pad_buf[];
extern Pad pad_data;
extern Pad prev_pad_data;
extern spi_t *spiPad;
extern int pad_time;

void rwPad(uint8_t* sendData, uint8_t* rcvData, uint8_t dataLength);
void spiInit();
int  updatePad();
int  checkAN();
void getButton();
uint8_t checkBTN(uint16_t comparisonData);
void getAnalogStick();
void VS_C2ToSerial();
bool existsPadInput();

#endif /* SPI_H */
