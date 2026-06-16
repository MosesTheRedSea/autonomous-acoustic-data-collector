/**
 * ESP32で使用するメモリマップに関する定義
 * 
 */
#ifndef Wrc058_MEMMAP_H
#define Wrc058_MEMMAP_H

#include "vs_wrc058_i2c.h"
#include "vs_wrc058_rs485.h"
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

//メモリマップ 
#define MAP_SIZE 0x100

//メモリマップ読み書きマクロ
#define R_MU8(x)  (*(uint8_t  *)(&wheel.memMap[x]))
#define R_MU16(x) (*(uint16_t *)(&wheel.memMap[x]))
#define R_MU32(x) (*(uint32_t *)(&wheel.memMap[x]))

#define R_MS8(x)  (*(int8_t   *)(&wheel.memMap[x]))
#define R_MS16(x) (*(int16_t  *)(&wheel.memMap[x]))
#define R_MS32(x) (*(int32_t  *)(&wheel.memMap[x]))




//SerialやBluetooth,Wi-Fiで受信した命令を格納する構造体
struct cmd{
  uint8_t cmdType;   //(読込)r, （書込）w
  uint8_t addr;     //読み書きするメモリマップの先頭アドレス
  uint8_t *value;   //書き込む値
  uint8_t valueCount;      //書き込む値の長さ
  uint8_t readLength;   //読みこみで先頭アドレスから何バイト読み込むか 0x00~0xff
};

enum memmapAddrIWS{
  MU16_SYSNAME  = 0x00,  /* R  システム名     */
  MU16_FIRMREV  = 0x02,  /* R  ファームウェアリビジョン  */
  MU32_TRIPTIME = 0x04,  /* R  経過時間(ms)  */
  MU16_WDT      =	0x08,	 /* R/W	ウォッチドッグタイマー(ms) */
/*                              0で何もしない、0xffffでモータ停止、それ以外でカウントダウン */
	MU8_CR0       =	0x0c,	 /* R	b0: 1でLED消灯 */
  MU8_M_MODE    = 0x0d,  /* R	 0xFD->モータ速度制御モード/0x01->位置制御*/
  MU16_POWOFF_T = 0x0e,  /* R/W	 msec後に電源が切れる */
  MU8_O_EN      = 0x10,  /* R/W  b0: 1でCH0出力イネーブル  */
/**/                     /* R/W  b1: 1でCH1出力イネーブル  */
/**/                     /* R/W  b2: 1でCH2出力イネーブル  */
/**/                     /* R/W  b3: 1でCH3出力イネーブル  */
  MU8_TRIG      = 0x11,  /* R/W  b0: 1でA_POS0をT_POS0に加算*/
/**/                     /* R/W  b1: 1でA_POS1をT_POS1に加算*/
/**/                     /* R/W  b2: 1でM_POS0とT_POS0をクリア */
/**/                     /* R/W  b3: 1でM_POS1とT_POS1をクリア */
/**/                     /* R/W  b4: 1で位置制御の処理を開始 */                    
  MU16_SD_VI    = 0x12,  /* R/W  シャットダウン電圧*/
  MU16_OD_DI    = 0x14,  /* R/W  対応するM_DIが1ならMU8_O_EN=0になる*/
  MS16_IM_TEMP0 = 0x16,  /* R モータCH0の温度[℃] */
  MS16_IM_TEMP1 = 0x18,  /* R モータCH1の温度[℃] */
  MS16_IM_TEMP2 = 0x1a,  /* R モータCH2の温度[℃] */
  MS16_IM_TEMP3 = 0x1c,  /* R モータCH3の温度[℃] */
  MU16_FB_VP0   = 0x20,  /* R/W  CH0速度比例ゲイン */
  MU16_FB_VP1   = 0x22,  /* R/W  CH1速度比例ゲイン */
  MU16_FB_VP2   = 0x24,  /* R/W  CH2速度比例ゲイン */
  MU16_FB_VP3   = 0x26,  /* R/W  CH3速度比例ゲイン */
  MU16_FB_VI0   = 0x28,  /* R/W  CH0速度積分ゲイン */
  MU16_FB_VI1   = 0x2a,  /* R/W  CH1速度積分ゲイン */
  MU16_FB_VI2   = 0x2c,  /* R/W  CH2速度積分ゲイン */
  MU16_FB_VI3   = 0x2e,  /* R/W  CH3速度積分ゲイン */
  MS16_FB_PP0   = 0x30,  /* R/W  CH0位置制御比例ゲイン */
  MS16_FB_PP1   = 0x32,  /* R/W  CH1位置制御比例ゲイン */
  MS16_FB_PP2   = 0x34,  /* R/W  CH2位置制御比例ゲイン */
  MS16_FB_PP3   = 0x36,  /* R/W  CH3位置制御比例ゲイン */
  MU16_P_CREF0	= 0x38,  /* R/W	CH0最大電流値(0x1000 = 10A) */
  MU16_P_CREF1	= 0x3a,  /* R/W	CH1最大電流値(0x1000 = 10A) */
  MU16_P_CREF2	= 0x3c,  /* R/W	CH2最大電流値(0x1000 = 10A) */
  MU16_P_CREF3	= 0x3e,  /* R/W	CH3最大電流値(0x1000 = 10A) */
  MU16_P_SLIMX  = 0x40,  /* R/W ローバーのX方向最大速度 */
  MU16_P_SLIMY  = 0x42,  /* R/W ローバーのY方向最大速度 */
  MU16_P_RLIM   = 0x44,  /* R/W ローバーの最大旋回速度  */
  MU16_P_ALIM   = 0x46,  /* R/W ローバーの最大加減速度  */
  MS32_IM_POS0  = 0x50,  /* R CH0の測定された位置 */
  MS32_IM_POS1  = 0x54,  /* R CH1の測定された位置 */
  MS32_IM_POS2  = 0x58,  /* R CH2の測定された位置 */
  MS32_IM_POS3  = 0x5c,  /* R CH3の測定された位置 */
  MS32_IM_SPD0  = 0x60,  /* R CH0の測定された速度 */
  MS32_IM_SPD1  = 0x64,  /* R CH1の測定された速度 */
  MS32_IM_SPD2  = 0x68,  /* R CH2の測定された速度 */
  MS32_IM_SPD3  = 0x6c,  /* R CH3の測定された速度 */
  MS16_IM_CUR0  = 0x70,  /* R CH0の測定された電流 */
  MS16_IM_CUR1  = 0x72,  /* R CH1の測定された電流 */
  MS16_IM_CUR2  = 0x74,  /* R CH2の測定された電流 */
  MS16_IM_CUR3  = 0x76,  /* R CH3の測定された電流 */
  MU16_IM_DI    = 0x80,  /* R デジタル入力 */
  MU16_M_VI    = 0x82,  /* R バッテリー電圧 */
  MS16_S_XS     = 0x90,  /* R/W X方向移動速度指令値 */
  MS16_S_YS     = 0x92,  /* R/W Y方向移動速度指令値 */
  MS16_S_ZS     = 0x94,  /* R/W 旋回速度指令値 */
  
  MS16_S_LWS    = 0x96,  /* R/W  速度制御：左車輪速度[mm/s] */
  MS16_S_RWS    = 0x98   /* R/W  速度制御：右車輪速度[mm/s] */
  
};

enum memmapAddr{
  //MU16_SYSNAME  = 0x00,  /* R  システム名     */
  //MU16_FIRMREV  = 0x02,  /* R  ファームウェアリビジョン  */
  //MU32_TRIPTIME = 0x04,  /* R  経過時間(ms)  */
  //MU16_WDT      =	0x08,	 /* R/W	ウォッチドッグタイマー(ms) */
/*                              0で何もしない=従来互換、0xffffでモータ停止、それ以外でカウントダウン */
	//MU8_CR0       =	0x0c,	 /* R	b0: 1でブラシレスモータのホールセンサカウントを位置として使用, 0でエンコーダカウント値を使用 */
  /*                          b1: 1でLED消灯 */
  MU8_MODE      = 0x0d,  /* R	b3-0:起動時に認識したDIP-SW */
  //MU16_POWOFF_T = 0x0e,  /* R/W	 msec後に電源が切れる */
  //MU8_O_EN      = 0x10,  /* R/W  b0: 1でCH0出力イネーブル  */
/**/                     /* R/W  b1: 1でCH1出力イネーブル  */
  //MU8_TRIG      = 0x11,  /* R/W  b0: 1でA_POS0をT_POS0に加算*/
/**/                     /* R/W  b1: 1でA_POS1をT_POS1に加算*/
/**/                     /* R/W  b2: 1でM_POS0とT_POS0をクリア */
/**/                     /* R/W  b3: 1でM_POS1とT_POS1をクリア */
/**/                     /* R/W  b4: 1で位置制御の処理を開始 */                    
  //MU16_SD_VI    = 0x12,  /* R/W  シャットダウン電圧*/
  //MU16_OD_DI    = 0x14,  /* R/W  対応するM_DIが1ならMU8_O_EN=0になる*/
  MU16_SPD_T0   = 0x16,  /* R/W  速度を計算するための基準時間(ms)*/
  MU16_DLY485   = 0x18,  /* R/W  RS485の送信ディレイ(バイト数)*/
  MU8_CSUM_EN	  = 0x1d,	 /* R/W	 reserved */
	MU8_CSUM_BUF0	= 0x1e,	 /* R/W	 reserved */
	MU8_CSUM_BUF1	= 0x1f,	 /* R/W	 reserved */
  MS16_FB_PG0   = 0x20,  /* R/W  比例ゲイン */
  MS16_FB_PG1   = 0x22,  /* R/W  比例ゲイン */
  MU16_FB_ALIM0 = 0x24,  /* R/W  加速度上限 */
  MU16_FB_ALIM1 = 0x26,  /* R/W  加速度上限 */
  MU16_FB_DLIM0 = 0x28,  /* R/W  減速度上限 */
  MU16_FB_DLIM1 = 0x2a,  /* R/W  減速度上限 */
  MU16_FB_OLIM0 = 0x2c,  /* R/W  出力上限  100%が0x1000*/
  MU16_FB_OLIM1 = 0x2e,  /* R/W  出力上限  100%が0x1000*/
  MU16_FB_PCH0  = 0x30,  /* R/W	 パンチ		  */
  MU16_FB_PCH1  = 0x32,  /* R/W	 パンチ		  */
  MS32_T_POS0   = 0x40,  /* R/W  目標エンコーダ値  */
  MS32_T_POS1   = 0x44,  /* R/W  目標エンコーダ値  */
  MS32_A_POS0   = 0x48,  /* R/W  T_POSへの加算値  */
  MS32_A_POS1   = 0x4c,  /* R/W  T_POSへの加算値  */
  MS16_T_OUT0   = 0x50,  /* R/W  出力オフセット */
  MS16_T_OUT1   = 0x52,  /* R/W  出力オフセット */
  MU16_T_CREF0	= 0x58,  /* R	最大電流値(0x1000 = 22A) */
  MU16_T_CREF1	= 0x5a,  /* R	最大電流値(0x1000 = 22A) */
  MU16_T_CLIM0	= 0x5c,  /* R	電流制限値(0x1000 = 22A) */
  MU16_T_CLIM1	= 0x5e,	 /* R	電流制限値(0x1000 = 22A) */
  MS32_M_POS0   = 0x60,  /* R  測定されたエンコーダ値 */
  MS32_M_POS1   = 0x64,  /* R  測定されたエンコーダ値 */
  MS16_M_SPD0   = 0x68,  /* R  測定された速度 M_POS0の差分*/
  MS16_M_SPD1   = 0x6a,  /* R  測定された速度 M_POS1の差分*/
  MS16_M_OUT0   = 0x6c,  /* R  計算されたモータ出力値 100%が0x1000*/
  MS16_M_OUT1   = 0x6e,  /* R  計算されたモータ出力値 100%が0x1000*/
  MU16_M_DI     = 0x7e,  /* R  b0-7:CN5, b8:PWR-BTN  */
  MU16_M_AN0    = 0x80,  /* R  (reserved)アナログ入力0 */
  MU16_M_AN1    = 0x82,  /* R  (reserved)アナログ入力1 */
  MU16_M_AN2    = 0x84,  /* R  (reserved)アナログ入力2 */
  MU16_M_AN3    = 0x86,  /* R  (reserved)アナログ入力3 */
  MU16_M_AN4    = 0x88,  /* R  (reserved)アナログ入力4 */
  MU16_M_AN5    = 0x8a,  /* R  (reserved)アナログ入力5 */
  MU16_M_AN6    = 0x8c,  /* R  (reserved)アナログ入力6 */
  MU16_M_AN7    = 0x8e,  /* R  (reserved)アナログ入力7 ※0x80-8fは予約のみ、現在機能はしていません。*/
  //MU16_M_VI     = 0x90,  /* R  入力電圧 3.3V*9=29.7Vが0x0fff(VS-WRC058の場合) */
  MU16_M_CUR0   = 0xa0,  /* R	 計測された電流値(0x1000 = 22A) */
  MU16_M_CUR1   = 0xa2,  /* R	 計測された電流値(0x1000 = 22A) */
  MS16_P_DIS    = 0xb0,  /* R/W  位置制御：移動距離[mm] */
  MS16_P_RAD    = 0xb2,  /* R/W  位置制御：旋回量[mrad]   */
  MS16_P_ACC    = 0xb4,  /* R/W  位置制御：加速度[mm/s^2]   */
  MS16_P_SPD    = 0xb6,  /* R/W  位置制御：移動速度[mm/s] */
  MU8_P_STTS    = 0xb8,  /* R/W  0x01で位置制御中、0x00で目標地点到達 */
  MU8_M_STTS    = 0xb9,  /* R    b0:1でWDT発動中、0で正常 */
  //MS16_S_XS     = 0xba,  /* R/W  速度制御：移動速度[mm/s] */
  //MS16_S_ZS     = 0xbc,  /* R/W  速度制御：旋回速度[mrad/s] */ 
  
  //MS16_S_LWS    = 0xac,  /* R/W  速度制御：左車輪速度[mm/s] */
  //MS16_S_RWS    = 0xae   /* R/W  速度制御：右車輪速度[mm/s] */
  
};

//メモリマップ初期値
//0x10から0x8fまでのメモリマップ初期化用
extern const uint8_t initialMemmap[MAP_SIZE];


//Wrc058のメモリマップクラス
class Wrc05x{
  public:
  Wrc05x(uint8_t id);
  uint8_t devId;
  uint8_t memMap[MAP_SIZE];
  uint8_t writeFlag[MAP_SIZE];
  uint8_t memUpCounter;


  virtual void initMemmap(double cutOffLevel);
  void memMapClean();
  virtual int readMemmap(uint8_t addr, uint8_t readLength);
  int readAll();
  virtual int writeMemmap(uint8_t addr, uint8_t data[], uint8_t writeLength);
  uint8_t getAddr();

  void checkMsg(String rcvMsg);
  void checkMsg(String rcvMsg, WiFiClient* client);
  void checkMsg(String rcvMsg, uint8_t viaBle);
  void checkMsg(String rcvMsg, WiFiClient* client, uint8_t viaBlt);
  void sendMap2pc(struct cmd rcvCmd);
  void sendMap2pc(struct cmd rcvCmd, WiFiClient* client);
  void sendMap2pc(struct cmd rcvCmd, WiFiClient* client, uint8_t viaBle);
  void setWriteMapViaMsg(struct cmd rcvCmd);

  int8_t   s8Map(uint8_t addr);
  int8_t   s8Map(uint8_t addr, int8_t data);
  uint8_t  u8Map(uint8_t addr);
  uint8_t  u8Map(uint8_t addr, uint8_t data);
  int16_t  s16Map(uint8_t addr);
  int16_t  s16Map(uint8_t addr, int16_t data);
  uint16_t u16Map(uint8_t addr);
  uint16_t u16Map(uint8_t addr, uint16_t data);
  int32_t  s32Map(uint8_t addr);
  int32_t  s32Map(uint8_t addr, int32_t data);
  uint32_t u32Map(uint8_t addr);
  uint32_t u32Map(uint8_t addr, uint32_t data);

  uint8_t checkWriteFlag(uint8_t addr);

  double getVin();

};

class Wrc051 : public Wrc05x{
  public:
  Wrc051(HardwareSerialRS485* io,uint8_t id);

  bool isAbleToUseCheckSum();
  bool isCheckSumEnabled();
  void updateSysMem();

  int sendWriteMapTime;

  void initMemmap(double cutOffLevel);
  int readMemmap(uint8_t addr, uint8_t readLength);
  int writeMemmap(uint8_t addr, uint8_t data[], uint8_t writeLength);

  int write4Byte(uint8_t addr, int32_t data);
  int write2Byte(uint8_t addr, int16_t data);
  int write1Byte(uint8_t addr, uint8_t data);

  void sendWriteMap();

  private:
  HardwareSerialRS485* iostream_;

};

class Wrc058 : public Wrc05x{
  public:
  Wrc058(uint8_t id);

  void initMemmap(double cutOffLevel);
  int readMemmap(uint8_t addr, uint8_t readLength);
  int writeMemmap(uint8_t addr, uint8_t data[], uint8_t writeLength);

};



extern Wrc058 wheel;
extern Wrc058 steer;

//extern static char hexToA[16];
uint8_t cToHex(uint8_t c);
int checkI2cAddrOfMsg(String rcvMsg, int rcvMsgCount);
int checkI2cAddrOfMsg(String rcvMsg, int rcvMsgCount, uint8_t viaBle);

//ウォッチドッグタイマ設定値
extern uint16_t wdTime;

#endif /* MEMMAP_H */


