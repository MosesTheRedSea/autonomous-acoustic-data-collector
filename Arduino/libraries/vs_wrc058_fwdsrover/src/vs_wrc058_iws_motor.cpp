#include "vs_wrc058_iws_motor.h"
#include <iostream>
#include <vector>
#include <algorithm>    
#include <iterator>  
#include <Arduino.h>
#include <driver/uart.h>

HardwareSerial SerialIWS(1);
HardwareSerial SerialIWS_ST(2);

IwsInwheelMotor iMtrFR(&SerialIWS, 0x01);
IwsInwheelMotor iMtrFL(&SerialIWS, 0x02);
IwsInwheelMotor iMtrRL(&SerialIWS, 0x03);
IwsInwheelMotor iMtrRR(&SerialIWS, 0x04);
IwsInwheelMotor iMtrSFR(&SerialIWS_ST, 0x01);
IwsInwheelMotor iMtrSFL(&SerialIWS_ST, 0x02);
IwsInwheelMotor iMtrSRL(&SerialIWS_ST, 0x03);
IwsInwheelMotor iMtrSRR(&SerialIWS_ST, 0x04);


unsigned char reverse_bit8(unsigned char x)
{
	x = ((x & 0x55) << 1) | ((x & 0xAA) >> 1);
	x = ((x & 0x33) << 2) | ((x & 0xCC) >> 2);
	return (x << 4) | (x >> 4);
}



IwsInwheelMotor::IwsInwheelMotor(SERIAL_CLASS* io, uint8_t id){
    iostream_ = io;
    id_ = id;
}

void IwsInwheelMotor::setRoverType(uint8_t type){
  if(type == 1){  //メガローバーVer.3.0 H40A
    wheel_circumference_ = 0.43982;;
  }
}

void IwsInwheelMotor::setDEPin(int dePin){
  dePinNr = dePin;
  //dePinSt = false;
  pinMode(dePin, OUTPUT);
  digitalWrite(dePin,false);
}

void IwsInwheelMotor::onDE(){
  digitalWrite(dePinNr, true);
  //dePinSt = true;
}

void IwsInwheelMotor::offDE(){
  digitalWrite(dePinNr, false);
  //dePinSt = false;
}

bool IwsInwheelMotor::beginTransmission(){

  //送信中の場合はリターン
  if(digitalRead(dePinNr)){
      return false;
  }

  //DEピンをHIGHに
  onDE();

  return true;
}

void IwsInwheelMotor::endTransmission(){
    
  //送信終了まで待機
  iostream_->flush();

  while(SerialIWS.available()){
    SerialIWS.read();
  }

  //DEピンをLOWに
  offDE();

  return;
}


void IwsInwheelMotor::changeBaud(){
  uint8_t send_baud[10] = {id_, 0x51, 0x10, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00};
  uint8_t save_s1[10]   = {id_, 0x51, 0x30, 0x61, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};

  readWriteMTR(send_baud);
  delay(50);
  readWriteMTR(save_s1);
  delay(50);

}

void IwsInwheelMotor::init(uint8_t type){
  uint8_t send_mode[10] = {id_, 0x51, 0x70, 0x17, 0x00, 0x00, 0x00, 0x00, 0xFD, 0x00};
  //uint8_t send_mode[10] = {id_, 0x51, 0x70, 0x17, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00};
  uint8_t send_en[10] = {id_, 0x52, 0x70, 0x19, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00};
  //uint8_t send_rpm[10] = {id_, 0x52, 0x70, 0xB1, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00};
  uint8_t read_acom[10] = {id_, 0xA0, 0x70, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t read_alim[10] = {id_, 0xA0, 0x70, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t read_aram[10] = {id_, 0xA0, 0x70, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t send_caram[10] = {id_, 0x52, 0x70, 0x19, 0x00, 0x00, 0x00, 0x00, 0x86, 0x00};
  uint8_t send_kpp[10] = {id_, 0x52, 0x70, 0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t send_kp[10] = {id_, 0x52, 0x70, 0xB3, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00};
  uint8_t send_ki[10] = {id_, 0x52, 0x70, 0xB4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t send_pspd[10] = {id_, 0x52, 0x70, 0x9D, 0x00, 0x00, 0x00, 0x01, 0x5E, 0x00};
  uint8_t send_aspd[10] = {id_, 0x54, 0x70, 0x99, 0x00, 0x00, 0x00, 0x00, 0xC9, 0x00};
  uint8_t send_dspd[10] = {id_, 0x54, 0x70, 0x9a, 0x00, 0x00, 0x00, 0x00, 0xC9, 0x00};

  uint8_t send_ewdt[10] = {id_, 0x51, 0x30, 0x10, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
  uint8_t send_twdt[10] = {id_, 0x54, 0x30, 0x11, 0x00, 0x00, 0x00, 0x03, 0xE8, 0x00};

  //readWriteMTR(read_aram);
  //delay(50);
  readWriteMTR(send_mode);
  //delay(50);
  readWriteMTR(send_caram);
  //delay(50);
  readWriteMTR(send_en);
  //delay(50);
/*  readWriteMTR(read_acom);
  delay(50);
  readWriteMTR(read_alim);
  delay(50);
  readWriteMTR(read_aram);
  delay(50);
*/  readWriteMTR(send_kpp);
  //delay(50);
  readWriteMTR(send_kp);
  //delay(50);
  readWriteMTR(send_ki);
  //delay(50);
  readWriteMTR(send_pspd);
  //delay(50);
  readWriteMTR(send_aspd);
  //delay(50);
  readWriteMTR(send_dspd);
  //delay(50);
  //readWriteMTR(send_twdt);
  //delay(50);
  //readWriteMTR(send_ewdt);
  //delay(50);

  setRoverType(type);

}

uint32_t IwsInwheelMotor::readMtr(uint16_t addr){
  uint8_t read_com[10] = {id_, 0xA0, 0x70, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  read_com[2] = addr >> 8;
  read_com[3] = addr;

  std::vector<uint8_t> tmp;
  tmp = readWriteMTR(read_com);

  uint32_t read_val = 0;
  read_val += (uint8_t)tmp[5] << 24;
  read_val += (uint8_t)tmp[6] << 16;
  read_val += (uint8_t)tmp[7] << 8;
  read_val += (uint8_t)tmp[8];

  return read_val;
}

void IwsInwheelMotor::writeMtr(uint16_t addr, uint8_t length, uint8_t* data){
  uint8_t write_com[10] = {id_, 0x51, 0x70, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  write_com[2] = addr >> 8;
  write_com[3] = addr;

  if(length == 1){
    write_com[1] = 0x51;
    write_com[8] = *data;
    readWriteMTR(write_com);

  }else if(length == 2){
    write_com[1] = 0x52;
    write_com[7] = *(data+1);
    write_com[8] = *data;
    readWriteMTR(write_com);

  }else if(length == 4){
    write_com[1] = 0x54;
    write_com[5] = *(data+3);
    write_com[6] = *(data+2);
    write_com[7] = *(data+1);
    write_com[8] = *data;
    readWriteMTR(write_com);

  }

  return;
}

void IwsInwheelMotor::calcCheckSum(uint8_t* data){
  int i = 0;
  uint16_t sum = 0;
  for(i = 0; i < 9; i++){
    sum += data[i];
  }

  data[9] = (uint8_t)sum;

  return;

}

std::vector<uint8_t> IwsInwheelMotor::readWriteMTR(uint8_t* data){

  calcCheckSum(data);

  int i = 0;

  while(beginTransmission() == false){
    delayMicroseconds(200);
  }
  //for(i = 0; i < 10; i++){
    iostream_->write(data, 10);
  //}
  endTransmission();

  std::vector<uint8_t> tmp{};

  int j = 0;
  while(iostream_->available() < 10){
    delayMicroseconds(200);
    if(j > 50){
      
      for(i = 0; i < 10; i++){
        tmp.push_back(0);
      }
      printf("no return com:%x / addr:%x%x / id:%d \n", data[1], data[2], data[3], id_);
      
      return tmp;
    }
    j++;

  }

  

  while(iostream_->available()){
    tmp.push_back(iostream_->read());
  }

  return tmp;
}

//目標速度[m/s]からrpm指令値を計算
double IwsInwheelMotor::calcRPMCom(double iv_com){

  double rpm = 0.0;

  rpm = iv_com*60.0 / wheel_circumference_;

  return rpm;


}


//rpm指令値から目標速度指令値を計算
int32_t IwsInwheelMotor::calcTSpd(double rpm){

  int32_t im_com = 0;

  im_com = (rpm*512*4096)/1875;

  return im_com;

}

//目標速度指令値を送信
void IwsInwheelMotor::sendMTRSpd(int32_t im_com){
  uint8_t send_com[10] = {id_, 0x54, 0x70, 0xB2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  send_com[5] = im_com >> 24;
  send_com[6] = im_com >> 16;
  send_com[7] = im_com >> 8;
  send_com[8] = im_com;

  readWriteMTR(send_com);

  return;
}

//ゲインの更新
void IwsInwheelMotor::updateGains(uint16_t sp, uint16_t si, uint16_t pp){
  uint8_t send_sp[10] = {id_, 0x52, 0x70, 0xB3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t send_si[10] = {id_, 0x52, 0x70, 0xB4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t send_pp[10] = {id_, 0x52, 0x70, 0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  send_sp[7] = sp >> 8;
  send_sp[8] = sp;

  send_si[7] = si >> 8;
  send_si[8] = si;
  
  send_pp[7] = pp >> 8;
  send_pp[8] = pp;

  readWriteMTR(send_sp);
  readWriteMTR(send_si);
  readWriteMTR(send_pp);

}

void IwsInwheelMotor::sendMTRPos(double iv_com){
  double e_v_com;  //速度指令値[ecnt/sec]
  e_v_com = iv_com*enc_per_mm*1000.0;  

  //前回から今回の処理までの経過時間
  
  uint32_t new_micros = micros(); //[μsec]
  double elapsed_time = (new_micros - prev_micros); //[μsec]
  if(elapsed_time < 0){
    elapsed_time = (new_micros + (UINT32_MAX - prev_micros)); //[μsec]
  }

  //逐次速度指令値から逐次加算値を求める
  static double i_buf_enc_com;
  i_buf_enc_com += (e_v_com*elapsed_time)/1000000.0;

  uint8_t send_t_pos[10] = {id_, 0x54, 0x70, 0x9F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  send_t_pos[5] = (int32_t)i_buf_enc_com >> 24;
  send_t_pos[6] = (int32_t)i_buf_enc_com >> 16;
  send_t_pos[7] = (int32_t)i_buf_enc_com >> 8;
  send_t_pos[8] = (int32_t)i_buf_enc_com;

  readWriteMTR(send_t_pos);


  i_buf_enc_com -=  (int32_t)i_buf_enc_com;

  prev_micros = new_micros;
}


