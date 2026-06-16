/**
 * VS-WRC058用ライブラリの各ファイルをまとめてincludeするためのヘッダ
 * およびユーティリティ的な関数集
 */
#ifndef WRC058_H
#define WRC058_H

#include "vs_wrc058_memmap.h"
#include "vs_wrc058_i2c.h"
#include "vs_wrc058_serial.h"
#include "vs_wrc058_motor.h"
#include "vs_wrc058_iws_motor.h"
#include "vs_wrc058_spi.h"
#include "vs_wrc058_wifi.h"
#include "vs_wrc058_ble_pad.h"

//ローバータイプの定義
#define FWDSROVER_X40A 1

enum wrc058StatusCode{
    NO_INPUT     = 0,          //入力無し or 無効な入力
    HTTP_ACCES   = 1,          //1~2はHTTPアクセスによる有効なメモリマップ操作命令
    HTTP_PAD     = 3,          //HTTPコントローラからのボタン入力
    //HTTP_SYSTEM  = WIFI_VIN,   //HTMLコントローラに電圧を表示するためのアクセス =4
    //HTTP_BAD     = BAD_REQUEST, //HTTPアクセスによるbad request =5    
    SERIAL_ACCES = 6,          //有線シリアル経由による有効なメモリマップ操作命令
    BLE_ACCES    = 7,
    BTC_ACCES    = 8,
    ROS_CTRL     = 9,
    ROS_ERR      = 10,
    PAD_INPUT    = 11

};

extern volatile bool isInterrupt;        //タイマー割り込み発報フラグ
extern volatile bool isInterrupt_s;      //タイマー割り込み発報フラグ（ステアリング）
extern portMUX_TYPE timerMux;
extern portMUX_TYPE timerMuxS;

void chkPadInput();

void setLPOW_EN(bool lpow);
void setLPOW_ON();
void setLPOW_OFF();

void setMPOW_EN(bool mpow);
void setMPOW_ON();
void setMPOW_OFF();

void setESP485_DE(bool de);
void setESP485_DE_ON();
void setESP485_DE_OFF();

void initP_BTN();
void chkP_BTN();

void initV_IN();
int getV_IN();
void chkVLevel();

int chkPowOffT();

void IRAM_ATTR onTimer();
void IRAM_ATTR onTimerS();
void setupInterruptTimer();

void ledInit();
void LED(int cmd);
void chkLEDFlag();

void setDIN(uint8_t rover_type);
void chkDIN(uint8_t rover_type);

void setTimeNow();


#endif /* WRC058 */
