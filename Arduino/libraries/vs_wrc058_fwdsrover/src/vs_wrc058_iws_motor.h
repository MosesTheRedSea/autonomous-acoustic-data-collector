/**
 * ESP32からIWSシリーズ　インホイールモータを制御するライブラリ
 * 
 */
#ifndef ESP32_IWSM_H
#define ESP32_IWSM_H

#include <Arduino.h>
#include <HardwareSerial.h>

#define SERIAL_CLASS HardwareSerial

//class IwsInwheelMotor;

enum iwsaddress{
  MS8_P_TYPE   = 0x7017,  /* R/W モータ動作モード */
  /**/                   /*     0xFD : 速度制御 */
  /**/                   /*     0x01 : 位置制御 */
  MS16_C_CTR   = 0x7019,  /* R/W モータ制御コマンド*/
  /**/                   /*     0x06 : モータフリー */
  /**/                   /*     0x0F : モータ通電 */
  /**/                   /*     0x86 : アラート開放 */
  MU16_S_ERR   = 0x7011,  /* R モータエラー */
  MS32_T_POS   = 0x709F,  /* R/W 相対目標位置 */
  MU32_P_PSPD  = 0x7098,  /* R/W 位置制御時速度指令値 */
  MU32_P_PALIM = 0x7099,  /* R/W 位置制御時台形制御加速度指令値 */
  MU32_P_PDLIM = 0x709A,  /* R/W 位置制御時台形制御減速度指令値 */
  MS32_T_SPD   = 0x70B2,  /* R/W 速度制御時目標速度 */
  MU16_P_CLIM  = 0x70E2,  /* R/W 電流制限値 */
  MS16_FB_PP   = 0x7094,  /* R/W 位置制御時比例ゲイン */
  MU16_FB_SP   = 0x70B3,  /* R/W 速度制御時比例ゲイン */
  MU16_FB_SI   = 0x70B4,  /* R/W 速度制御時積分ゲイン */
  MS32_M_POS   = 0x7071,  /* R 計測された位置 */
  MS32_M_SPD   = 0x7077,  /* R 計測された速度 */
  MS16_M_CUR   = 0x7072,  /* R 計測された電流 */
  MS16_M_DEG   = 0x7002,  /* R 計測された温度 */
  MU8_P_ID     = 0x100C   /* R/W RS485デバイスID */


};

class IwsInwheelMotor{
public:
  IwsInwheelMotor(SERIAL_CLASS* io, uint8_t id);

  void setRoverType(uint8_t type);
  void setDEPin(int dePin=21);    //DEピンをセット、DE制御用のマルチタスクをセットアップする
  void onDE();                    //DEピンをオンにする
  void offDE();                   //DEピンをオフにする
  bool beginTransmission();       //通信を始める
  void endTransmission();         //通信を終わる
  //bool getMsg();
  void changeBaud();
  void init(uint8_t type);
  uint32_t readMtr(uint16_t addr);
  void writeMtr(uint16_t addr, uint8_t length, uint8_t* data);
  void calcCheckSum(uint8_t* data);
  std::vector<uint8_t> readWriteMTR(uint8_t* data);
  double calcRPMCom(double iv_com);
  int32_t calcTSpd(double rpm);
  void sendMTRSpd(int32_t im_com);
  void updateGains(uint16_t sp, uint16_t si, uint16_t pp);
  void sendMTRPos(double iv_com);

protected:
  uint8_t id_ = 0x01;
  SERIAL_CLASS* iostream_;
  double wheel_circumference_  = 0.43982;  //φ140mm
  double enc_per_mm = 5;
  uint32_t prev_micros = micros();

  int dePinNr = 12;
  bool dePinSt = false;

};

extern HardwareSerial SerialIWS;
extern HardwareSerial SerialIWS_ST;

extern IwsInwheelMotor iMtrFL;
extern IwsInwheelMotor iMtrFR;
extern IwsInwheelMotor iMtrRL;
extern IwsInwheelMotor iMtrRR;
extern IwsInwheelMotor iMtrSFL;
extern IwsInwheelMotor iMtrSFR;
extern IwsInwheelMotor iMtrSRL;
extern IwsInwheelMotor iMtrSRR;


#endif /* IWSM_H */