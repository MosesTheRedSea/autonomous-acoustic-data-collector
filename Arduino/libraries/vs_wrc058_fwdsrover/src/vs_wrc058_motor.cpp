#include "vs_wrc058_motor.h"
#include "vs_wrc058_memmap.h"
#include "vs_wrc058_spi.h"
#include <Arduino.h>
#include <math.h>
#include <cstdint>

uint8_t run_mode = RUN_TURN;

double  ctl_v_com[4] = {0.0, 0.0, 0.0, 0.0};//制御器からの速度指令値
double  v_com[4] = {0.0, 0.0, 0.0, 0.0};    //加減速制限後の速度指令値
int16_t m_com[4] = {0, 0, 0, 0};    //左右モータへの出力指令値

int32_t enc[4] = {0, 0, 0, 0};
int32_t old_enc[4] = {0, 0, 0, 0};


double sum_v[4][5] = {{0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}};
double avr_v[4] = {0.0, 0.0, 0.0, 0.0};

double v_enc[4]  = {0.0, 0.0, 0.0, 0.0};
double v_diff[4] = {0.0, 0.0, 0.0, 0.0};
double prev_v_diff[4] = {0.0, 0.0, 0.0, 0.0};
double prev2_v_diff[4] = {0.0, 0.0, 0.0, 0.0};
int16_t prev_m[4] = {0, 0, 0, 0};

uint32_t pid_time = 0;
uint32_t pos_time = 0;

float std_motor_param[] = { 1.0,  //最大並進速度[m/s]
                            3.14, //最大旋回速度[rad/s]
                            1000, //速度からモータ出力値への変換係数  //2600
                            1.6,  //ホイールPゲイン
                            0.2,  //ホイールIゲイン
                            1.6,  //ホイールDゲイン
                            30,   //コントローラデッドゾーン(絶対値)
                            127,  //コントローラ入力最大値(絶対値)
                            1.5,  //タイヤ回転速度に対する最大加速度[m/s^2]
                            12.56,  //ステアリング最大回転速度[rad/s]
                            37.68, //ステアリング回転速度に対する最大加速度[rad/s^2]
                            9.42   //ステアリング位置Pゲイン

                          };

float motor_param[] = { 1.0,  //最大並進速度[m/s]
                        3.14, //最大旋回速度[rad/s]
                        1000, //速度からモータ出力値への変換係数
                        1.6,  //Pゲイン
                        0.2,  //Iゲイン
                        1.6,  //Dゲイン
                        30,   //コントローラデッドゾーン(絶対値)
                        127,  //コントローラ入力最大値(絶対値)
                        1.5,  //タイヤ回転速度に対する最大加速度[m/s^2]
                        12.56,  //ステアリング最大回転速度[rad/s]
                        37.68, //ステアリング回転速度に対する最大加速度[rad/s^2]
                        9.42   //ステアリング位置Pゲイン
                      };

double  steer_range[4] = {0.0, 0.0, 0.0, 0.0};
int32_t steer_max[4] = {0, 0, 0, 0};
double  steer_center[4] = {0.0, 0.0, 0.0, 0.0};
double  steer_enc_per_rad[4] = {10000.0, 10000.0, 10000.0, 10000.0};
double  e_steer_speed_max[4] = {0.0, 0.0, 0.0, 0.0};
double  steer_pos_com_rad[4] = {0.0, 0.0, 0.0, 0.0};
int32_t  e_steer_pos_com[4]   = {0, 0, 0, 0};
double  d_pos_com[4]         = {0.0, 0.0, 0.0, 0.0};
double  steer_pos_now[4]     = {0.0, 0.0, 0.0, 0.0};
uint8_t init_step = 0;

const double root_2 = sqrt(2.0);


double rover_d = 0.250/2.0;  //車輪間距離（車幅方向） H40A -> 0.250/2.0[m]
double rover_hb = 0.250/2.0; //車輪間距離（ホイールベース方向）H40A -> 0.250/2.0[m]
double tire_circumference = 477.52;   //タイヤ直径　H40A -> 439.82[mm]
double enc_counts_per_turn = 4096;  //タイヤ1回転当たりのエンコーダカウント値 
double enc_per_mm = enc_counts_per_turn/tire_circumference; //単位距離当たりのエンコーダ値
uint8_t rover_type = 1;
//車両サイズなどの設定
void setBodyDetail(int type){

  if(type == 1){  //4WDSローバーX40A
    rover_d = 0.250/2.0;
    rover_hb = 0.250/2.0;
    tire_circumference = 439.82;
    enc_counts_per_turn = 4096;
    enc_per_mm = enc_counts_per_turn/tire_circumference;
    rover_type = type;
  }

}

//出力ON/OFFを設定： 0->両方OFF 1->CH0のみON 2->CH1のみON 3->両方ON
int setO_EN(int data){
  setO_EN(data, &wheel);
  setO_EN(data, &steer);

  return data;
}

int setO_EN(int data, Wrc058* memmap){

  if(memmap->u8Map(MU8_O_EN) != data){
    memmap->u8Map(MU8_O_EN, data);
  }

  return data;
}

//出力ON/OFFの設定を確認
int getO_EN(){
  int return_value = 0;
  return_value += getO_EN(&wheel);

  return return_value;
}

int getO_EN(Wrc058* memmap){
  //memmap->readMemmap(MU8_O_EN, 1);

  return memmap->u8Map(MU8_O_EN);
}

//O_ENをもとにモータのON/OFFを切り替える
void updO_EN(){

  uint8_t motorON[2]  = {0x0F, 0x00};
  uint8_t motorOFF[2] = {0x06, 0x00};

  if(wheel.writeFlag[MU8_O_EN]){
    wheel.writeFlag[MU8_O_EN] = 0x00;
    
    switch(wheel.u8Map(MU8_O_EN)){
      case ON_ON :
        iMtrFL.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        iMtrFR.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        iMtrRL.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        iMtrRR.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        break;
      
      case ON_OFF :
        iMtrFL.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        iMtrFR.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        iMtrRL.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrRR.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        break;

      case OFF_ON :
        iMtrFL.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrFR.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrRL.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        iMtrRR.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        break;
      
      case OFF_OFF :
        iMtrFL.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrFR.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrRL.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrRR.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        break;

      default :
        iMtrFL.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrFR.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrRL.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrRR.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);

      
    }
    
  }

  return;
}

void updO_EN_S(){

  uint8_t motorON[2]  = {0x0F, 0x00};
  uint8_t motorOFF[2] = {0x06, 0x00};

  if(steer.writeFlag[MU8_O_EN]){
    steer.writeFlag[MU8_O_EN] = 0x00;
    
    switch(steer.u8Map(MU8_O_EN)){
      case ON_ON :
        iMtrSFL.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        iMtrSFR.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        iMtrSRL.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        iMtrSRR.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        break;
      
      case ON_OFF :
        iMtrSFL.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        iMtrSFR.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        iMtrSRL.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrSRR.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        break;

      case OFF_ON :
        iMtrSFL.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrSFR.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrSRL.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        iMtrSRR.writeMtr(MS16_C_CTR, 2, &motorON[0]);
        break;
      
      case OFF_OFF :
        iMtrSFL.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrSFR.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrSRL.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrSRR.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        break;

      default :
        iMtrSFL.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrSFR.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrSRL.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);
        iMtrSRR.writeMtr(MS16_C_CTR, 2, &motorOFF[0]);

      
    }
    
  }

  return;
}

//直接制御モードと位置制御モードを切り替え：　1->直接制御 0->位置制御
int setCtrlMode(int ctrl_mode){
  setCtrlMode(ctrl_mode, &wheel);
  //setCtrlMode(ctrl_mode, &steer);

  return ctrl_mode;
}

int setCtrlMode(int ctrl_mode, Wrc058* memmap){

  //現在の制御モードを確認
  if(getCtrlMode(memmap)){
    //現在はSPD制御
    if(ctrl_mode == MODE_POS){ //位置制御に変更なら

      //位置制御に設定
      memmap->u8Map(MU8_M_MODE, 0x01);
    }
  }else{
    //現在は位置制御
    if(ctrl_mode == MODE_SPD){ //SPD制御に変更なら

      //直接制御に設定
      memmap->u8Map(MU8_M_MODE, 0xFD);
    } 
  }
  
  return ctrl_mode;
}

//現在の制御モードを取得
int getCtrlMode(){
  int return_value = 0;;
  return_value += getCtrlMode(&wheel);
  return_value += getCtrlMode(&steer);

  return return_value;
}

int getCtrlMode(Wrc058* memmap){
  //memmap->readMemmap(MS16_FB_PG0, 4);
  if(memmap->u8Map(MU8_M_MODE) == 0x01){
    return MODE_POS;
  }else{
    return MODE_SPD;
  }
}

//モータ動作モードのモータへのupd
void updIWSMode(){

  if(wheel.writeFlag[MU8_M_MODE]){
    wheel.writeFlag[MU8_M_MODE] = 0x00;

    iMtrFL.writeMtr(MS8_P_TYPE, 1, &wheel.memMap[MU8_M_MODE]);
    iMtrFR.writeMtr(MS8_P_TYPE, 1, &wheel.memMap[MU8_M_MODE]);
    iMtrRL.writeMtr(MS8_P_TYPE, 1, &wheel.memMap[MU8_M_MODE]);
    iMtrRR.writeMtr(MS8_P_TYPE, 1, &wheel.memMap[MU8_M_MODE]);
  }

  if(steer.writeFlag[MU8_M_MODE]){
    steer.writeFlag[MU8_M_MODE] = 0x00;

    iMtrSFL.writeMtr(MS8_P_TYPE, 1, &wheel.memMap[MU8_M_MODE]);
    iMtrSFR.writeMtr(MS8_P_TYPE, 1, &wheel.memMap[MU8_M_MODE]);
    iMtrSRL.writeMtr(MS8_P_TYPE, 1, &wheel.memMap[MU8_M_MODE]);
    iMtrSRR.writeMtr(MS8_P_TYPE, 1, &wheel.memMap[MU8_M_MODE]);
  }

  return;
}


//cylinderモータenable
void steeringMotorEnable(IwsInwheelMotor* inst_iMtr){
  uint8_t motorON[2]  = {0x0F, 0x00};
  inst_iMtr->writeMtr(MS16_C_CTR, 2, &motorON[0]);
  
}

//cylinderモータdisable
void steeringMotorDisable(IwsInwheelMotor* inst_iMtr){
  uint8_t motorOFF[2] = {0x06, 0x00};
  inst_iMtr->writeMtr(MS16_C_CTR, 2, &motorOFF[0]);

}

//cylinderモータ、エンコーダリセット
void resetSteeringMotorEnc(IwsInwheelMotor* inst_iMtr){
  uint8_t reset_enc = 0x01;
  inst_iMtr->writeMtr(0x70AC, 1, &reset_enc);

}

//セットアップ確認
bool isInitSteerFin(){
  if(init_step > 5){
    return true;
  }else{
    return false;
  }

  return false;
}

//セットアップ
int initSteer(){

  if(init_step == 0){
    //速度制御モードに設定
    Serial.println("start init step0");
    uint8_t sp_mode = 0xFD;
    iMtrSFR.writeMtr(MS8_P_TYPE, 1, &sp_mode);
    iMtrSFL.writeMtr(MS8_P_TYPE, 1, &sp_mode);
    iMtrSRL.writeMtr(MS8_P_TYPE, 1, &sp_mode);
    iMtrSRR.writeMtr(MS8_P_TYPE, 1, &sp_mode);

    steeringMotorEnable(&iMtrSFR);
    steeringMotorEnable(&iMtrSFL);
    steeringMotorEnable(&iMtrSRL);
    steeringMotorEnable(&iMtrSRR);

    steer.u16Map(MU16_FB_VP0, 0x0030);
    steer.u16Map(MU16_FB_VI0, 0x0001);
    steer.u16Map(MU16_FB_VP1, 0x0030);
    steer.u16Map(MU16_FB_VI1, 0x0001);
    steer.u16Map(MU16_FB_VP2, 0x0030);
    steer.u16Map(MU16_FB_VI2, 0x0001);
    steer.u16Map(MU16_FB_VP3, 0x0030);
    steer.u16Map(MU16_FB_VI3, 0x0001);

    steer.writeFlag[MU16_FB_VP0] = 0x00;
    iMtrSFR.writeMtr(MU16_FB_SP, 2, &steer.memMap[MU16_FB_VP0]);
    steer.writeFlag[MU16_FB_VI0] = 0x00;
    iMtrSFR.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI0]);
    steer.writeFlag[MU16_FB_VP1] = 0x00;
    iMtrSFL.writeMtr(MU16_FB_SP, 2, &steer.memMap[MU16_FB_VP1]);
    steer.writeFlag[MU16_FB_VI1] = 0x00;
    iMtrSFL.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI1]);
    steer.writeFlag[MU16_FB_VP2] = 0x00;
    iMtrSRL.writeMtr(MU16_FB_SP, 2, &steer.memMap[MU16_FB_VP2]);
    steer.writeFlag[MU16_FB_VI2] = 0x00;
    iMtrSRL.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI2]);
    steer.writeFlag[MU16_FB_VP3] = 0x00;
    iMtrSRR.writeMtr(MU16_FB_SP, 2, &steer.memMap[MU16_FB_VP3]);
    steer.writeFlag[MU16_FB_VI3] = 0x00;
    iMtrSRR.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI3]);


    sp_mode = iMtrSFR.readMtr(MS8_P_TYPE);
    if(sp_mode == 0xFD){
      sp_mode = iMtrSFL.readMtr(MS8_P_TYPE);
      if(sp_mode == 0xFD){
        sp_mode = iMtrSRL.readMtr(MS8_P_TYPE);
        if(sp_mode == 0xFD){
          sp_mode = iMtrSRR.readMtr(MS8_P_TYPE);
          if(sp_mode == 0xFD){
            init_step++;
            return init_step;
          }else{
            Serial.print("SRR: ");
            Serial.println(sp_mode, HEX);
          }
        }else{
          Serial.print("SRL: ");
          Serial.println(sp_mode, HEX);
        }
      }else{
        Serial.print("SFL: ");
        Serial.println(sp_mode, HEX);
      }
    }else{
      Serial.print("SFR: ");
      Serial.println(sp_mode, HEX);
    }

  }else if(init_step == 1){
    //負側最低点まで移動して停止
    Serial.println("start step1");

    int32_t init_enc_s[4] = {0, 0, 0, 0};
    static int32_t last_enc_s[4] = {0, 0, 0, 0};
    static int same_cnt_s[4] = {0, 0, 0, 0};

    for(uint8_t i = 0; i < 4; i++){
      getSteerEncoder(i);
      init_enc_s[i] = steer.s32Map(MS32_IM_POS0+i*4);

      if(abs(init_enc_s[i] - last_enc_s[i]) <= 5 ){
        same_cnt_s[i]++;
        //printf("same val No. %d: %d/5\n",i,same_cnt_s[i]);
      }else{
        if(same_cnt_s[i] <=5){
          same_cnt_s[i] = 0;
        }
      }
      printf("No. %d: %d vs %d\n",i,init_enc_s[i],last_enc_s[i]);
      last_enc_s[i] = init_enc_s[i];
      
    }

    double high_sp = -0.8;
    double middle_sp = -0.4;
    double low_sp  = -0.2;

    if(same_cnt_s[0] > 5){
      steer.u16Map(MU16_FB_VI0, 0x0000);
      steer.writeFlag[MU16_FB_VI0] = 0x00;
      iMtrSFR.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI0]);
      iMtrSFR.sendMTRSpd(iMtrSFR.calcTSpd(SRADS_MRPM(low_sp)));
    }else if(same_cnt_s[0] > 2){
      iMtrSFR.sendMTRSpd(iMtrSFR.calcTSpd(SRADS_MRPM(middle_sp)));
    }else{
      iMtrSFR.sendMTRSpd(iMtrSFR.calcTSpd(SRADS_MRPM(high_sp)));
    }
    if(same_cnt_s[1] > 5){
      steer.u16Map(MU16_FB_VI1, 0x0000);
      steer.writeFlag[MU16_FB_VI1] = 0x00;
      iMtrSFL.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI1]);
      iMtrSFL.sendMTRSpd(iMtrSFL.calcTSpd(SRADS_MRPM(low_sp)));
    }else if(same_cnt_s[1] > 2){
      iMtrSFL.sendMTRSpd(iMtrSFL.calcTSpd(SRADS_MRPM(middle_sp)));
    }else{
      iMtrSFL.sendMTRSpd(iMtrSFL.calcTSpd(SRADS_MRPM(high_sp)));
    }
    if(same_cnt_s[2] > 5){
      steer.u16Map(MU16_FB_VI2, 0x0000);
      steer.writeFlag[MU16_FB_VI2] = 0x00;
      iMtrSRL.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI2]);
      iMtrSRL.sendMTRSpd(iMtrSRL.calcTSpd(SRADS_MRPM(low_sp)));
    }else if(same_cnt_s[2] > 2){
      iMtrSRL.sendMTRSpd(iMtrSRL.calcTSpd(SRADS_MRPM(middle_sp)));
    }else{
      iMtrSRL.sendMTRSpd(iMtrSRL.calcTSpd(SRADS_MRPM(high_sp)));
    }
    if(same_cnt_s[3] > 5){
      steer.u16Map(MU16_FB_VI3, 0x0000);
      steer.writeFlag[MU16_FB_VI3] = 0x00;
      iMtrSRR.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI3]);
      iMtrSRR.sendMTRSpd(iMtrSRR.calcTSpd(SRADS_MRPM(low_sp)));
    }else if(same_cnt_s[3] > 2){
      iMtrSRR.sendMTRSpd(iMtrSRR.calcTSpd(SRADS_MRPM(middle_sp)));
    }else{
      iMtrSRR.sendMTRSpd(iMtrSRR.calcTSpd(SRADS_MRPM(high_sp)));
    }

    if(same_cnt_s[0]>5 && same_cnt_s[1]>5 && same_cnt_s[2]>5 && same_cnt_s[3]>5){
      printf("init 1 | %d| %d| %d| %d|\n", steer.s32Map(MS32_IM_POS0), steer.s32Map(MS32_IM_POS1), steer.s32Map(MS32_IM_POS2), steer.s32Map(MS32_IM_POS3));
      init_step++;
      return init_step;
    }

    

  }else if(init_step == 2){
    //エンコーダーリセット
    Serial.println("start step2");
    steeringMotorDisable(&iMtrSFR);
    steeringMotorDisable(&iMtrSFL);
    steeringMotorDisable(&iMtrSRL);
    steeringMotorDisable(&iMtrSRR);

    resetSteeringMotorEnc(&iMtrSFR);
    resetSteeringMotorEnc(&iMtrSFL);
    resetSteeringMotorEnc(&iMtrSRL);
    resetSteeringMotorEnc(&iMtrSRR);

    steeringMotorEnable(&iMtrSFR);
    steeringMotorEnable(&iMtrSFL);
    steeringMotorEnable(&iMtrSRL);
    steeringMotorEnable(&iMtrSRR);

    steer.s32Map(MS32_IM_POS0, iMtrSFR.readMtr(MS32_M_POS));
    steer.s32Map(MS32_IM_POS1, iMtrSFL.readMtr(MS32_M_POS));
    steer.s32Map(MS32_IM_POS2, iMtrSRL.readMtr(MS32_M_POS));
    steer.s32Map(MS32_IM_POS3, iMtrSRR.readMtr(MS32_M_POS));

    iMtrSFR.sendMTRSpd(iMtrSFR.calcTSpd(SRADS_MRPM(0.0)));
    iMtrSFL.sendMTRSpd(iMtrSFL.calcTSpd(SRADS_MRPM(0.0)));
    iMtrSRL.sendMTRSpd(iMtrSRL.calcTSpd(SRADS_MRPM(0.0)));
    iMtrSRR.sendMTRSpd(iMtrSRR.calcTSpd(SRADS_MRPM(0.0)));



    steer.u16Map(MU16_FB_VP0, 0x0030);
    steer.u16Map(MU16_FB_VI0, 0x0001);
    steer.u16Map(MU16_FB_VP1, 0x0030);
    steer.u16Map(MU16_FB_VI1, 0x0001);
    steer.u16Map(MU16_FB_VP2, 0x0030);
    steer.u16Map(MU16_FB_VI2, 0x0001);
    steer.u16Map(MU16_FB_VP3, 0x0030);
    steer.u16Map(MU16_FB_VI3, 0x0001);

    steer.writeFlag[MU16_FB_VP0] = 0x00;
    iMtrSFR.writeMtr(MU16_FB_SP, 2, &steer.memMap[MU16_FB_VP0]);
    steer.writeFlag[MU16_FB_VI0] = 0x00;
    iMtrSFR.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI0]);
    steer.writeFlag[MU16_FB_VP1] = 0x00;
    iMtrSFL.writeMtr(MU16_FB_SP, 2, &steer.memMap[MU16_FB_VP1]);
    steer.writeFlag[MU16_FB_VI1] = 0x00;
    iMtrSFL.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI1]);
    steer.writeFlag[MU16_FB_VP2] = 0x00;
    iMtrSRL.writeMtr(MU16_FB_SP, 2, &steer.memMap[MU16_FB_VP2]);
    steer.writeFlag[MU16_FB_VI2] = 0x00;
    iMtrSRL.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI2]);
    steer.writeFlag[MU16_FB_VP3] = 0x00;
    iMtrSRR.writeMtr(MU16_FB_SP, 2, &steer.memMap[MU16_FB_VP3]);
    steer.writeFlag[MU16_FB_VI3] = 0x00;
    iMtrSRR.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI3]);


    init_step++;
    return init_step;

  }else if(init_step == 3){
    //モーターを正側限界まで移動
    Serial.println("start step3");

    int32_t init_enc_s[4] = {0, 0, 0, 0};
    static int32_t last_enc_s[4] = {0, 0, 0, 0};
    static int same_cnt_s[4] = {0, 0, 0, 0};

    for(uint8_t i = 0; i < 4; i++){
      getSteerEncoder(i);
      init_enc_s[i] = steer.s32Map(MS32_IM_POS0+i*4);

      if(abs(init_enc_s[i] - last_enc_s[i]) <= 5){
        same_cnt_s[i]++;
      }else{
        if(same_cnt_s[i] <=5){
          same_cnt_s[i] = 0;
        }
      }
      printf("No. %d: %d vs %d\n",i,init_enc_s[i],last_enc_s[i]);
      last_enc_s[i] = init_enc_s[i];
      
    }

    double high_sp = 0.8;
    double middle_sp = 0.4;
    double low_sp  = 0.2;

    if(same_cnt_s[0] > 5){
      steer.u16Map(MU16_FB_VI0, 0x0000);
      steer.writeFlag[MU16_FB_VI0] = 0x00;
      iMtrSFR.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI0]);
      iMtrSFR.sendMTRSpd(iMtrSFR.calcTSpd(SRADS_MRPM(low_sp)));
    }else if(same_cnt_s[0] > 2){
      iMtrSFR.sendMTRSpd(iMtrSFR.calcTSpd(SRADS_MRPM(middle_sp)));
    }else{
      iMtrSFR.sendMTRSpd(iMtrSFR.calcTSpd(SRADS_MRPM(high_sp)));
    }
    if(same_cnt_s[1] > 5){
      steer.u16Map(MU16_FB_VI1, 0x0000);
      steer.writeFlag[MU16_FB_VI1] = 0x00;
      iMtrSFL.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI1]);
      iMtrSFL.sendMTRSpd(iMtrSFL.calcTSpd(SRADS_MRPM(low_sp)));
    }else if(same_cnt_s[1] > 2){
      iMtrSFL.sendMTRSpd(iMtrSFL.calcTSpd(SRADS_MRPM(middle_sp)));
    }else{
      iMtrSFL.sendMTRSpd(iMtrSFL.calcTSpd(SRADS_MRPM(high_sp)));
    }
    if(same_cnt_s[2] > 5){
      steer.u16Map(MU16_FB_VI2, 0x0000);
      steer.writeFlag[MU16_FB_VI2] = 0x00;
      iMtrSRL.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI2]);
      iMtrSRL.sendMTRSpd(iMtrSRL.calcTSpd(SRADS_MRPM(low_sp)));
    }else if(same_cnt_s[2] > 2){
      iMtrSRL.sendMTRSpd(iMtrSRL.calcTSpd(SRADS_MRPM(middle_sp)));
    }else{
      iMtrSRL.sendMTRSpd(iMtrSRL.calcTSpd(SRADS_MRPM(high_sp)));
    }
    if(same_cnt_s[3] > 5){
      steer.u16Map(MU16_FB_VI3, 0x0000);
      steer.writeFlag[MU16_FB_VI3] = 0x00;
      iMtrSRR.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI3]);
      iMtrSRR.sendMTRSpd(iMtrSRR.calcTSpd(SRADS_MRPM(low_sp)));
    }else if(same_cnt_s[3] > 2){
      iMtrSRR.sendMTRSpd(iMtrSRR.calcTSpd(SRADS_MRPM(middle_sp)));
    }else{
      iMtrSRR.sendMTRSpd(iMtrSRR.calcTSpd(SRADS_MRPM(high_sp)));
    }

    if(same_cnt_s[0]>5 && same_cnt_s[1]>5 && same_cnt_s[2]>5 && same_cnt_s[3]>5){
      printf("init 3 | %d| %d| %d| %d|\n", steer.s32Map(MS32_IM_POS0), steer.s32Map(MS32_IM_POS1), steer.s32Map(MS32_IM_POS2), steer.s32Map(MS32_IM_POS3));
      init_step++;
      return init_step;
    }

  }else if(init_step == 4){
    //ステア最大値およびcenterを計算し、目標値をcenterにセット
    Serial.println("start step4");
    for(uint8_t i = 0; i < 4; i++){
      steer_max[i] = steer.s32Map(MS32_IM_POS0 + i*4);            //[cnt]
      steer_enc_per_rad[i] = steer_max[i]/(DEG_RAD(340));           //[cnt/rad]
      steer_center[i] = (steer_max[i]/2.0)/steer_enc_per_rad[i];  //[rad]

      steer_pos_com_rad[i] = steer_center[i];                     //[rad]

      printf("No. %d: max-> %d e/r-> %lf center-> %lf pos_com-> %lf\n",i,steer_max[i],steer_enc_per_rad[i], steer_center[i], steer_pos_com_rad[i]);
    }
    
    init_step++;
    return init_step;

  }else if(init_step == 5){
    //ゲイン等を設定し、モータをenable
    Serial.println("start step5");
/*
    //モーター位置制御モードを使用する場合の設定値
    wrc058.u16Map(MU16_FB_VP2, 0x0000);
    wrc058.u16Map(MU16_FB_VI2, 0x0004);
    wrc058.u16Map(MS16_FB_PP2, 0x0000);

    iMtrC.writeMtr(MU16_FB_SP, 2, &wrc058.memMap[MU16_FB_VP2]);
    iMtrC.writeMtr(MU16_FB_SI, 2, &wrc058.memMap[MU16_FB_VI2]);
    iMtrC.writeMtr(MS16_FB_PP, 2, &wrc058.memMap[MS16_FB_PP2]);

    uint8_t motor_posc = 0x01;
    iMtrC.writeMtr(MS8_P_TYPE, 1, &motor_posc);

    cylinderMotorEnableAPos();

    uint32_t a_lim = 402;
    uint8_t w_a_lim[4] = {0x00, 0x00, 0x00, 0x00};

    w_a_lim[3] = (int32_t)a_lim >> 24;
    w_a_lim[2] = (int32_t)a_lim >> 16;
    w_a_lim[1] = (int32_t)a_lim >> 8;
    w_a_lim[0] = (int32_t)a_lim;

    iMtrC.writeMtr(0x7099, 4, &w_a_lim[0]);
    iMtrC.writeMtr(0x709A, 4, &w_a_lim[0]);
*/
/*
    cylinderMotorEnable();
    uint8_t motor_posc;
    motor_posc = (uint8_t)iMtrC.readMtr(MS8_P_TYPE);
    if(motor_posc == 0xFD){ //0x01
      init_step++;
      Serial.println(init_step);
      Serial.println("finish cylinder init");

      
      Serial.println(iMtrC.readMtr(0x709D));        //初期値350
      Serial.println(iMtrC.readMtr(0x7099));        //初期値201
      Serial.println(iMtrC.readMtr(0x709A)/67.10);  //初期値201
      
    }
*/
    init_step++;

  }

  return init_step;

}


//ステアリングモータのエンコーダ値を取得する
void getSteerEncoder(uint8_t motor_no){
  int32_t tmp_enc;

  switch(motor_no){
    case 0:
      tmp_enc = iMtrSFR.readMtr(MS32_M_POS);
      if(abs(steer.s32Map(MS32_IM_POS0) - tmp_enc) > steer_enc_per_rad[motor_no]){
        //printf("larger enc val : %d %d\n", motor_no, steer.s32Map(MS32_IM_POS0) - tmp_enc);
        //steer.s32Map(MS32_IM_POS0, tmp_enc);
      }else{
        steer.s32Map(MS32_IM_POS0, tmp_enc);
      }
      //steer.s32Map(MS32_IM_POS0, iMtrSFR.readMtr(MS32_M_POS));
      break;
    case 1:
      tmp_enc = iMtrSFL.readMtr(MS32_M_POS);
      if(abs(steer.s32Map(MS32_IM_POS1) - tmp_enc) > steer_enc_per_rad[motor_no]){
        //printf("larger enc val : %d %d\n", motor_no, steer.s32Map(MS32_IM_POS1) - tmp_enc);
        //steer.s32Map(MS32_IM_POS1, tmp_enc);
      }else{
        steer.s32Map(MS32_IM_POS1, tmp_enc);
      }
      //steer.s32Map(MS32_IM_POS1, iMtrSFL.readMtr(MS32_M_POS));
      break;
    case 2:
      tmp_enc = iMtrSRL.readMtr(MS32_M_POS);
      if(abs(steer.s32Map(MS32_IM_POS2) - tmp_enc) > steer_enc_per_rad[motor_no]){
        //printf("larger enc val : %d %d\n", motor_no, steer.s32Map(MS32_IM_POS2) - tmp_enc);
        //steer.s32Map(MS32_IM_POS2, tmp_enc);
      }else{
        steer.s32Map(MS32_IM_POS2, tmp_enc);
      }
      //steer.s32Map(MS32_IM_POS2, iMtrSRL.readMtr(MS32_M_POS));
      break;
    case 3:
      tmp_enc = iMtrSRR.readMtr(MS32_M_POS);
      if(abs(steer.s32Map(MS32_IM_POS3) - tmp_enc) > steer_enc_per_rad[motor_no]){
        //printf("larger enc val : %d %d\n", motor_no, steer.s32Map(MS32_IM_POS3) - tmp_enc);
        //steer.s32Map(MS32_IM_POS3, tmp_enc);
      }else{
        steer.s32Map(MS32_IM_POS3, tmp_enc);
      }
      //steer.s32Map(MS32_IM_POS3, iMtrSRR.readMtr(MS32_M_POS));

  }  
}

//ステアリング軸の現在の角度を取得
double getSteerRad(uint8_t motor_no){

  getSteerEncoder(motor_no);
  int32_t steer_enc = 0;
  steer_enc = steer.s32Map(MS32_IM_POS0 + motor_no*4);

  double steer_rad = steer_enc /steer_enc_per_rad[motor_no];

  return steer_rad;
}


//モーターの速度制御モードを用いてステアリングを目標位置に向けて移動
void moveSteerPosV(){

  //前回から今回の処理までの経過時間
  static uint32_t prev_micros = micros();
  uint32_t new_micros = micros(); //[μsec]
  double elapsed_time = (new_micros - prev_micros); //[μsec]
  if(elapsed_time < 0){
    elapsed_time = (new_micros + (UINT32_MAX - prev_micros)); //[μsec]
  }



  static double last_steer_v_com[4] = {0.0, 0.0, 0.0, 0.0};
  double steer_v_com[4] = {0.0, 0.0, 0.0, 0.0};
  double max_spd_variation = motor_param[MAX_STR_A] * elapsed_time /1000000.0;  //許容される回転速度増加量
  for(uint8_t i = 0; i < 4; i++){
    steer_pos_now[i] = getSteerRad(i);

    steer_v_com[i] = (steer_pos_com_rad[i] - steer_pos_now[i])*motor_param[STR_K_P]; //[rad/s]

    if(steer_v_com[i] > motor_param[MAX_STR_S]){
      steer_v_com[i] = motor_param[MAX_STR_S];
    }else if(steer_v_com[i] < -motor_param[MAX_STR_S]){
      steer_v_com[i] = -motor_param[MAX_STR_S];
    }else if(steer_v_com[i] >= 0.0 && steer_v_com[i] < 0.01){
      steer_v_com[i] = 0.01;
    }else if(steer_v_com[i] < 0.0 && steer_v_com[i] > -0.01){
      steer_v_com[i] = -0.01;
    }

    if(fabs(steer_v_com[i] - last_steer_v_com[i]) > max_spd_variation && fabs(steer_v_com[i]) > fabs(last_steer_v_com[i])){
      //加速オーバー
      if(steer_v_com[i] > last_steer_v_com[i]){
        //+方向加速オーバー
        steer_v_com[i] = last_steer_v_com[i] + max_spd_variation;
      }else{
        //-方向加速オーバー
        steer_v_com[i] = last_steer_v_com[i] - max_spd_variation;
      }
    }

    last_steer_v_com[i] = steer_v_com[i];

    //速度に応じたパラメータ書き換え
    if(steer_v_com[i] == 0.0){
      if(steer.u16Map(MU16_FB_VP0+i*2) != 0x0030){
        steer.u16Map(MU16_FB_VP0+i*2, 0x0030);
      }
      if(steer.u16Map(MU16_FB_VI0+i*2) != 0x0001){
        steer.u16Map(MU16_FB_VI0+i*2, 0x0001);
      }
    }else{
      if(steer.u16Map(MU16_FB_VP0+i*2) != 0x0030){
        steer.u16Map(MU16_FB_VP0+i*2, 0x0030);
      }
      if(steer.u16Map(MU16_FB_VI0+i*2) != 0x0002){
        steer.u16Map(MU16_FB_VI0+i*2, 0x0002);
      }
    }
  }

  if(steer.writeFlag[MU16_FB_VP0]){
    steer.writeFlag[MU16_FB_VP0] = 0x00;
    iMtrSFR.writeMtr(MU16_FB_SP, 2, &steer.memMap[MU16_FB_VP0]);
  }
  if(steer.writeFlag[MU16_FB_VI0]){
    steer.writeFlag[MU16_FB_VI0] = 0x00;
    iMtrSFR.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI0]);
  }

  if(steer.writeFlag[MU16_FB_VP1]){
    steer.writeFlag[MU16_FB_VP1] = 0x00;
    iMtrSFL.writeMtr(MU16_FB_SP, 2, &steer.memMap[MU16_FB_VP1]);
  }
  if(steer.writeFlag[MU16_FB_VI1]){
    steer.writeFlag[MU16_FB_VI1] = 0x00;
    iMtrSFL.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI1]);
  }

  if(steer.writeFlag[MU16_FB_VP2]){
    steer.writeFlag[MU16_FB_VP2] = 0x00;
    iMtrSRL.writeMtr(MU16_FB_SP, 2, &steer.memMap[MU16_FB_VP2]);
  }
  if(steer.writeFlag[MU16_FB_VI2]){
    steer.writeFlag[MU16_FB_VI2] = 0x00;
    iMtrSRL.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI2]);
  }

  if(steer.writeFlag[MU16_FB_VP3]){
    steer.writeFlag[MU16_FB_VP3] = 0x00;
    iMtrSRR.writeMtr(MU16_FB_SP, 2, &steer.memMap[MU16_FB_VP3]);
  }
  if(steer.writeFlag[MU16_FB_VI3]){
    steer.writeFlag[MU16_FB_VI3] = 0x00;
    iMtrSRR.writeMtr(MU16_FB_SI, 2, &steer.memMap[MU16_FB_VI3]);
  }

/*
  printf("No. 0: %3.2lf to %3.2lf p %lf\n",steer_pos_now[0], steer_pos_com_rad[0], steer_v_com[0]);
  printf("No. 1: %3.2lf to %3.2lf p %lf\n",steer_pos_now[1], steer_pos_com_rad[1], steer_v_com[1]);
  printf("No. 2: %3.2lf to %3.2lf p %lf\n",steer_pos_now[2], steer_pos_com_rad[2], steer_v_com[2]);
  printf("No. 3: %3.2lf to %3.2lf p %lf\n",steer_pos_now[3], steer_pos_com_rad[3], steer_v_com[3]);
*/

  



  iMtrSFR.sendMTRSpd(iMtrSFR.calcTSpd(SRADS_MRPM(steer_v_com[S_F_R])));
  iMtrSFL.sendMTRSpd(iMtrSFL.calcTSpd(SRADS_MRPM(steer_v_com[S_F_L])));
  iMtrSRL.sendMTRSpd(iMtrSRL.calcTSpd(SRADS_MRPM(steer_v_com[S_R_L])));
  iMtrSRR.sendMTRSpd(iMtrSRR.calcTSpd(SRADS_MRPM(steer_v_com[S_R_R])));

  //cylinderMotorEnable();
  
  prev_micros = new_micros;


}


//ゲームパッドの入力値を速度に変換
void aStick2V(){
  //デッドゾーン判定
  double pad_left_y;
  double pad_left_x;
  double pad_right_y;
  double pad_right_x;
  if(pad_data.left_stick.y >= motor_param[PAD_DEAD]){
    pad_left_y = (double)pad_data.left_stick.y - motor_param[PAD_DEAD];
  }else if(pad_data.left_stick.y <= -1.0*motor_param[PAD_DEAD]){
    pad_left_y = (double)pad_data.left_stick.y + motor_param[PAD_DEAD];
  }else{
    pad_left_y = 0.0;
  }

  if(pad_data.left_stick.x >= motor_param[PAD_DEAD]){
    pad_left_x = (double)pad_data.left_stick.x - motor_param[PAD_DEAD];
  }else if(pad_data.left_stick.x <= -1.0*motor_param[PAD_DEAD]){
    pad_left_x = (double)pad_data.left_stick.x + motor_param[PAD_DEAD];
  }else{
    pad_left_x = 0.0;
  }

  if(pad_data.right_stick.y >= motor_param[PAD_DEAD]){
    pad_right_y = (double)pad_data.right_stick.y - motor_param[PAD_DEAD];
  }else if(pad_data.right_stick.y <= -1.0*motor_param[PAD_DEAD]){
    pad_right_y = (double)pad_data.right_stick.y + motor_param[PAD_DEAD];
  }else{
    pad_right_y = 0.0;
  }

  if(pad_data.right_stick.x >= motor_param[PAD_DEAD]){
    pad_right_x = (double)pad_data.right_stick.x - motor_param[PAD_DEAD];
  }else if(pad_data.right_stick.x <= -1.0*motor_param[PAD_DEAD]){
    pad_right_x = (double)pad_data.right_stick.x + motor_param[PAD_DEAD];
  }else{
    pad_right_x = 0.0;
  }

  //左スティックの入力から、並進移動方向を求める
  double l_stick_vector;
  
  //右スティックの入力から、角速度に合わせた各輪の舵角を求める

  //右スティック入力->角速度
  double diff_max_dead = motor_param[PAD_MAX] - motor_param[PAD_DEAD];
  double omega;
  if(run_mode == RUN_HORIZONTAL){
    //omega = motor_param[MAX_RAD]*-1.0*pad_right_y/diff_max_dead;
    omega = motor_param[MAX_RAD]*pad_right_y/diff_max_dead;
  }else{
    //omega = motor_param[MAX_RAD]*-1.0*pad_right_x/diff_max_dead;
    omega = motor_param[MAX_RAD]*pad_right_x/diff_max_dead;
  }
  

  //左スティック->並進移動速度(絶対値)
  double translation_velocity;
  translation_velocity = sqrt(pow(pad_left_x/diff_max_dead,2.0) + pow(pad_left_y/diff_max_dead, 2.0))*motor_param[MAX_SPEED];


  //run_modeチェック
  if(run_mode == RUN_STOP){
    //静止
    l_stick_vector = 0.0;
    omega = 0.0;
    translation_velocity = 0.0;

  }else if(run_mode == RUN_TURN){
    //超信地旋回のみ
    l_stick_vector = 0.0;
    translation_velocity = 0.0;
  }else if(run_mode == RUN_VERTICAL){
    //縦移動モード
    if(pad_left_x != 0.0 || pad_left_y != 0.0){
      //l_stick_vector = atan2(-1.0*pad_left_x, pad_left_y);
      l_stick_vector = atan2(pad_left_x, pad_left_y);
      if(l_stick_vector<0.0){
        l_stick_vector += M_PI*2.0;
      }
    }else{
      l_stick_vector = 0.0;
    }

    if(l_stick_vector > DEG_RAD(90.0) && l_stick_vector <= DEG_RAD(180.0)){
      l_stick_vector += M_PI;
      translation_velocity = -1.0*translation_velocity;
    }else if(l_stick_vector > DEG_RAD(180.0) && l_stick_vector < DEG_RAD(270.0)){
      l_stick_vector -= M_PI;
      translation_velocity = -1.0*translation_velocity;
    }

    if(l_stick_vector == DEG_RAD(90.0) || l_stick_vector == DEG_RAD(270.0)){
      translation_velocity = 0.0;
    }

    if(translation_velocity < 0.0){
      omega = -1.0*omega;
    }else if(translation_velocity == 0.0){
      omega = 0.0;
    }
    



  }else if(run_mode == RUN_HORIZONTAL){
    //横移動モード
    if(pad_left_x != 0.0 || pad_left_y != 0.0){
      l_stick_vector = atan2(-1.0*pad_left_x, pad_left_y);
      l_stick_vector = atan2(pad_left_x, pad_left_y);
      if(l_stick_vector<0.0){
        l_stick_vector += M_PI*2.0;
      }
    }else{
      l_stick_vector = DEG_RAD(90.0);
    }

    if(l_stick_vector > DEG_RAD(180.0) && l_stick_vector <= DEG_RAD(270.0)){
      l_stick_vector += M_PI;
      translation_velocity = -1.0*translation_velocity;
    }else if(l_stick_vector > DEG_RAD(270.0) && l_stick_vector < DEG_RAD(360.0)){
      l_stick_vector -= M_PI;
      translation_velocity = -1.0*translation_velocity;
    }

    if(l_stick_vector == DEG_RAD(0.0) || l_stick_vector == DEG_RAD(180.0)){
      translation_velocity = 0.0;
    }

    if(translation_velocity < 0.0){
      omega = -1.0*omega;
    }else if(translation_velocity == 0.0){
      omega = 0.0;
    }

  }

  double steer_pos_com_rad_tmp[4] = {0.0, 0.0, 0.0, 0.0};
  //l_stick_vectorに基づき、各車輪の角度を求める
  if(run_mode == RUN_VERTICAL || run_mode == RUN_HORIZONTAL){
    //4WDSローバー座標上の進行方向Vectorをステアリング軸座標に置き換える
    steer_pos_com_rad_tmp[S_F_R] = steer_center[S_F_R] + (M_PI/4.0) - l_stick_vector;
    steer_pos_com_rad_tmp[S_F_L] = steer_center[S_F_L] - (M_PI/4.0) - l_stick_vector;
    steer_pos_com_rad_tmp[S_R_L] = steer_center[S_R_L] + (M_PI/4.0) - l_stick_vector;
    steer_pos_com_rad_tmp[S_R_R] = steer_center[S_R_R] - (M_PI/4.0) - l_stick_vector;


  }else{
    //並進移動入力なし
    steer_pos_com_rad_tmp[0] = steer_center[0];
    steer_pos_com_rad_tmp[1] = steer_center[1];
    steer_pos_com_rad_tmp[2] = steer_center[2];
    steer_pos_com_rad_tmp[3] = steer_center[3];
  }

  

  //旋回中心の旋回半径
  double center_turning_r;
  if(omega != 0.0){
    center_turning_r = translation_velocity/omega;

    if(center_turning_r > 0.0 && center_turning_r < rover_d-0.050){
      center_turning_r = rover_d-0.050;
    }else if(center_turning_r < 0.0 && center_turning_r > -1.0*rover_d+0.050){
      center_turning_r = -1.0*rover_d+0.050;
    }

  }else{
    //直進
    center_turning_r = 0.0;
  }

  //各輪の旋回半径を求める
  double turning_r[4];
  double turning_r_x[4];
  double turning_r_y[4];
  turning_r_x[S_F_L] = root_2*rover_d*cos(M_PI/4.0-l_stick_vector);
  turning_r_x[S_F_R] = root_2*rover_d*sin(M_PI/4.0-l_stick_vector);
  turning_r_x[S_R_L] = -1.0*root_2*rover_d*sin(M_PI/4.0-l_stick_vector);
  turning_r_x[S_R_R] = -1.0*root_2*rover_d*cos(M_PI/4.0-l_stick_vector);
  turning_r_y[S_F_L] = center_turning_r - root_2*rover_d*sin(M_PI/4.0 - l_stick_vector);
  turning_r_y[S_F_R] = center_turning_r + root_2*rover_d*cos(M_PI/4.0 - l_stick_vector);
  turning_r_y[S_R_L] = center_turning_r - root_2*rover_d*cos(M_PI/4.0 - l_stick_vector);
  turning_r_y[S_R_R] = center_turning_r + root_2*rover_d*sin(M_PI/4.0 - l_stick_vector);

  int i;
  if(omega != 0.0){
    for(i = 0; i < 4; i++){
      turning_r[i] = sqrt(pow(turning_r_x[i], 2.0) + pow(turning_r_y[i], 2.0));
      if(turning_r_y[i] < 0.0){
        turning_r[i] = -1.0*turning_r[i];
      }
    }
  }else{
    //直進
    for(i = 0; i < 4; i++){
      turning_r[i] = root_2*rover_d;
    }
  }
  

  //各輪の旋回成分による舵角
  double steer_by_turn[4];
  if(omega != 0.0 && (pad_left_x != 0.0 || pad_left_y != 0.0)){
    //並進移動入力があり、かつomegaが0.0でないとき
    for(i = 0; i < 4; i++){
      double rad_cow = atan2(turning_r_y[i], turning_r_x[i]);
      if(rad_cow >= 0.0){
        steer_by_turn[i] = (rad_cow - (M_PI/2.0));
      }else{
        steer_by_turn[i] = (rad_cow + (M_PI/2.0));
      }
      
    }
  }else{
    //超信地旋回or旋回なしのとき
    for(i = 0; i < 4; i++){
      steer_by_turn[i] = 0.0;
    }
  }

  //旋回中心が車体の内側に来た際にモータが反転しないようにする
  for(i = 0; i < 4; i++){
    if(center_turning_r > 0.0 && turning_r[i] < 0.0){
      steer_by_turn[i] += M_PI;
      turning_r[i] = -1.0*turning_r[i];
    }else if(center_turning_r < 0.0 && turning_r[i] > 0.0){
      steer_by_turn[i] -= M_PI;
      turning_r[i] = -1.0*turning_r[i];
    }
  }

  //並進移動による車輪角と旋回による舵角の和
  for(i = 0; i < 4; i++){
    steer_pos_com_rad_tmp[i] += steer_by_turn[i];
  }


  //各輪の速度
  if(omega != 0.0){
    for(i = 0; i < 4; i++){
      ctl_v_com[i] = turning_r[i]*omega;
    }
  }else{
    //旋回無し
    for(i = 0; i < 4; i++){
      ctl_v_com[i] = translation_velocity;
    }
  }

  //横移動モード時に車輪角度、回転方向を補正
  if(run_mode == RUN_HORIZONTAL /*&& translation_velocity != 0.0*/){
    steer_pos_com_rad_tmp[S_F_L] += M_PI;
    steer_pos_com_rad_tmp[S_R_R] += M_PI;
    ctl_v_com[F_L] = -1.0*ctl_v_com[F_L];
    ctl_v_com[R_R] = -1.0*ctl_v_com[R_R];
  }  


  //指令角を0-2PIの範囲に調整
  for(i = 0; i < 4; i++){
    if(steer_pos_com_rad_tmp[i] > M_PI*2.0){
      steer_pos_com_rad_tmp[i] = fmod(steer_pos_com_rad_tmp[i] ,(M_PI*2.0));
    }else if(steer_pos_com_rad_tmp[i] < 0.0){
      steer_pos_com_rad_tmp[i] += M_PI*2.0;
      while(steer_pos_com_rad_tmp[i] < 0.0){
        steer_pos_com_rad_tmp[i] += M_PI*2.0;
      }
    }
  }

  //1番2番の速度を反転
  ctl_v_com[1] = -1.0*ctl_v_com[1];
  ctl_v_com[2] = -1.0*ctl_v_com[2]; 

  

  //最大速度による制限
  if(fabs(ctl_v_com[F_L]) > motor_param[MAX_SPEED] || fabs(ctl_v_com[F_R]) > motor_param[MAX_SPEED] || fabs(ctl_v_com[R_L]) > motor_param[MAX_SPEED] || fabs(ctl_v_com[R_R]) > motor_param[MAX_SPEED]){
    double v_com_max = fabs(ctl_v_com[0]);
    for(i = 1; i < 4; i++){
      if(v_com_max < fabs(ctl_v_com[i])){
        v_com_max = fabs(ctl_v_com[i]);
      }
    }

    for(i = 0; i < 4; i++){
      ctl_v_com[i] /= (v_com_max/motor_param[MAX_SPEED]);
    }
  }


  /*
  for(i = 0; i < 4; i++){
    steer_pos_now[i] = getSteerRad(i);
  }
  */
  //静止状態への移行の場合、速度が0.0m/s以上の場合は、ステアリング旋回を行わない
  if(ctl_v_com[F_R] == 0.0 && ctl_v_com[F_L] == 0.0 && ctl_v_com[R_L] == 0.0 && ctl_v_com[R_R] == 0.0 && fabs(avr_v[F_R]) > 0.0 && fabs(avr_v[F_L]) > 0.0 && fabs(avr_v[R_L]) > 0.0 && fabs(avr_v[R_R]) > 0.0){
    return;
  }

  //ホイールモータ保護のため、1輪でも現在の角度と指示角度に20度以上の差があった場合、ホイールモータの速度指令値を0にする
  for(i = 0; i < 4; i++){
    if(fabs(steer_pos_now[i] - steer_pos_com_rad_tmp[i]) > DEG_RAD(40.0)){
      ctl_v_com[F_R] = 0.0;
      ctl_v_com[F_L] = 0.0;
      ctl_v_com[R_L] = 0.0;
      ctl_v_com[R_R] = 0.0;
    }
  }

  //printf("%lf | %lf\n", steer_pos_now[0] - steer_pos_com_rad_tmp[0], ctl_v_com[F_R]);

  //タイヤ保護のため、現在の速度が0.1m/s以上の場合、20度以上のステアリング旋回を行わない
  for(i = 0; i < 4; i++){
    if(fabs(avr_v[i]) >= 0.1 && fabs(steer_pos_com_rad_tmp[i] - steer_pos_com_rad[i]) >= DEG_RAD(40.0)){
      
    }else{
      steer_pos_com_rad[i] = steer_pos_com_rad_tmp[i];
    }
  }

}

//メモリマップ記載の速度値から、各輪の速度を算出する
void memCom2V(){
  double steer_pos_com_rad_tmp[4] = {0.0, 0.0, 0.0, 0.0};

  double mem_com_x = wheel.s16Map(MS16_S_XS)/1000.0;  //X方向速度指令値[m/s]
  double mem_com_y = wheel.s16Map(MS16_S_YS)/1000.0;  //Y方向速度指令値[m/s]
  double mem_com_z = wheel.s16Map(MS16_S_ZS)/1000.0;  //Z軸まわり旋回速度指令値[rad/s]


  //動作モードの判定
  if(mem_com_x == 0.0 && mem_com_y == 0.0){
    run_mode = RUN_TURN;
  }else if(fabs(mem_com_x) >= fabs(mem_com_y)){
    run_mode = RUN_VERTICAL;
  }else{
    run_mode = RUN_HORIZONTAL;
  }

  //並進移動方向を求める
  double translation_vector;
  if(mem_com_x != 0.0 || mem_com_y != 0.0){
    translation_vector = atan2(mem_com_y, mem_com_x);
    if(translation_vector<0.0){
      translation_vector += M_PI*2.0;
    }
  }else{
    translation_vector = 0.0;
  }

  //メモリマップの角速度指令値から、角速度に合わせた各輪の舵角を求める
  //角速度指令値->角速度
  double omega;
  omega = mem_com_z;
  if(omega > motor_param[MAX_RAD]){
    omega = motor_param[MAX_RAD];
  }else if( omega < -1.0*motor_param[MAX_RAD]){
    omega = -1.0*motor_param[MAX_RAD];
  }

  //X方向速度指令値とY方向速度指令値->並進移動速度(絶対値)
  double translation_velocity;
  //mem_com_xとmem_com_yに対して最大速度補正を掛けるべき
  translation_velocity = sqrt(pow(mem_com_x/motor_param[MAX_SPEED],2.0) + pow(mem_com_y/motor_param[MAX_SPEED], 2.0))*motor_param[MAX_SPEED];


  //run_modeチェック
  if(run_mode == RUN_STOP){
    //静止
    translation_vector = 0.0;
    omega = 0.0;
    translation_velocity = 0.0;

  }else if(run_mode == RUN_TURN){
    //超信地旋回のみ
    translation_vector = 0.0;
    translation_velocity = 0.0;
  }else if(run_mode == RUN_VERTICAL){
    //縦移動モード
    if(translation_vector > DEG_RAD(45.0) && translation_vector <= DEG_RAD(90.0)){
      //45-90deg
      translation_vector = DEG_RAD(45.0);
    }else if(translation_vector > DEG_RAD(90.0) && translation_vector < DEG_RAD(135.0)){
      //90-135deg
      translation_vector = DEG_RAD(135.0); 
    }else if(translation_vector > DEG_RAD(225.0) && translation_vector < DEG_RAD(270.0)){
      //225-270deg
      translation_vector = DEG_RAD(225.0);
    }else if(translation_vector >= DEG_RAD(270.0) && translation_vector < DEG_RAD(315.0)){
      //270-315deg
      translation_vector = DEG_RAD(315.0);
    }

    if(translation_vector >= DEG_RAD(135.0) && translation_vector <= DEG_RAD(180.0)){
      translation_vector += M_PI;
      translation_velocity = -1.0*translation_velocity;
    }else if(translation_vector > DEG_RAD(180.0) && translation_vector <= DEG_RAD(225.0)){
      translation_vector -= M_PI;
      translation_velocity = -1.0*translation_velocity;
    }

  }else if(run_mode == RUN_HORIZONTAL){
    //横移動モード
    if(translation_vector >= DEG_RAD(0.0) && translation_vector < DEG_RAD(45.0)){
      //45-90deg
      translation_vector = DEG_RAD(45.0);
    }else if(translation_vector > DEG_RAD(135.0) && translation_vector <= DEG_RAD(180.0)){
      //90-135deg
      translation_vector = DEG_RAD(135.0); 
    }else if(translation_vector > DEG_RAD(180.0) && translation_vector < DEG_RAD(225.0)){
      //225-270deg
      translation_vector = DEG_RAD(225.0);
    }else if(translation_vector > DEG_RAD(315.0) && translation_vector <= DEG_RAD(360.0)){
      //270-315deg
      translation_vector = DEG_RAD(315.0);
    }

    if(translation_vector >= DEG_RAD(225.0) && translation_vector <= DEG_RAD(270.0)){
      translation_vector += M_PI;
      translation_velocity = -1.0*translation_velocity;
    }else if(translation_vector > DEG_RAD(270.0) && translation_vector <= DEG_RAD(315.0)){
      translation_vector -= M_PI;
      translation_velocity = -1.0*translation_velocity;
    }

    if(translation_velocity == 0.0){
      translation_vector = 0.0;
    }

  }

  if(mem_com_x != 0.0 || mem_com_y != 0.0){
    //translation_vectorに基づき、各車輪の角度を求める
    //4WDSローバー座標上の進行方向Vectorをステアリング軸座標に置き換える
    steer_pos_com_rad_tmp[0] = steer_center[0] + (M_PI/4.0) - translation_vector;
    steer_pos_com_rad_tmp[1] = steer_center[1] - (M_PI/4.0) - translation_vector;
    steer_pos_com_rad_tmp[2] = steer_center[2] + (M_PI/4.0) - translation_vector;
    steer_pos_com_rad_tmp[3] = steer_center[3] - (M_PI/4.0) - translation_vector;

  }else{
    //並進移動なし
    steer_pos_com_rad_tmp[0] = steer_center[0];
    steer_pos_com_rad_tmp[1] = steer_center[1];
    steer_pos_com_rad_tmp[2] = steer_center[2];
    steer_pos_com_rad_tmp[3] = steer_center[3];
  }
  

  //旋回中心の旋回半径
  double center_turning_r;
  if(omega != 0.0){
    center_turning_r = translation_velocity/omega;
  }else{
    //直進
    center_turning_r = 0.0;
  }

  //各輪の旋回半径
  double turning_r[4];
  double turning_r_x[4];
  double turning_r_y[4];
  turning_r_x[S_F_L] = root_2*rover_d*cos(M_PI/4.0-translation_vector);
  turning_r_x[S_F_R] = root_2*rover_d*sin(M_PI/4.0-translation_vector);
  turning_r_x[S_R_L] = -1.0*root_2*rover_d*sin(M_PI/4.0-translation_vector);
  turning_r_x[S_R_R] = -1.0*root_2*rover_d*cos(M_PI/4.0-translation_vector);
  turning_r_y[S_F_L] = center_turning_r - root_2*rover_d*sin(M_PI/4.0 - translation_vector);
  turning_r_y[S_F_R] = center_turning_r + root_2*rover_d*cos(M_PI/4.0 - translation_vector);
  turning_r_y[S_R_L] = center_turning_r - root_2*rover_d*cos(M_PI/4.0 - translation_vector);
  turning_r_y[S_R_R] = center_turning_r + root_2*rover_d*sin(M_PI/4.0 - translation_vector);

  int i;
  if(omega != 0.0){
    for(i = 0; i < 4; i++){
      turning_r[i] = sqrt(pow(turning_r_x[i], 2.0) + pow(turning_r_y[i], 2.0));
      if(turning_r_y[i] < 0.0){
        turning_r[i] = -1.0*turning_r[i];
      }
    }
  }else{
    for(i = 0; i < 4; i++){
      turning_r[i] = 0.0;
    }
  }
  

  //各輪の旋回成分による舵角
  double steer_by_turn[4];
  if(omega != 0.0 && (mem_com_x != 0.0 || mem_com_y != 0.0)){
    //並進移動入力があり、かつomegaが0.0でないとき
    for(i = 0; i < 4; i++){
      double rad_cot = atan2(turning_r_y[i], turning_r_x[i]);
      if(rad_cot >= 0.0){
        //steer_by_turn[i] = -1.0*(rad_cot - (M_PI/2.0));
        steer_by_turn[i] = (rad_cot - (M_PI/2.0));
      }else{
        //steer_by_turn[i] = -1.0*(rad_cot + (M_PI/2.0));
        steer_by_turn[i] = (rad_cot + (M_PI/2.0));
      }
      
    }
  }else{
    //超信地旋回or旋回なしのとき
    for(i = 0; i < 4; i++){
      steer_by_turn[i] = 0.0;
    }
  }

  //並進移動による車輪角と旋回による舵角の和
  for(i = 0; i < 4; i++){
    steer_pos_com_rad_tmp[i] += steer_by_turn[i];
  }

  //各輪の速度
  if(omega != 0.0){
    for(i = 0; i < 4; i++){
      ctl_v_com[i] = turning_r[i]*omega;
    }
  }else{
    //旋回無し
    for(i = 0; i < 4; i++){
      ctl_v_com[i] = translation_velocity;
    }
  }


  //横移動モード時に車輪角度、回転方向を補正
  if(run_mode == RUN_HORIZONTAL && translation_velocity != 0.0){
    steer_pos_com_rad_tmp[S_F_L] += M_PI;
    steer_pos_com_rad_tmp[S_R_R] += M_PI;
    ctl_v_com[F_L] = -1.0*ctl_v_com[F_L];
    ctl_v_com[R_R] = -1.0*ctl_v_com[R_R];
  }  


  //指令角を0-2PIの範囲に調整
  for(i = 0; i < 4; i++){
    if(steer_pos_com_rad_tmp[i] > M_PI*2.0){
      steer_pos_com_rad_tmp[i] = fmod(steer_pos_com_rad_tmp[i] ,(M_PI*2.0));
    }else if(steer_pos_com_rad_tmp[i] < 0.0){
      steer_pos_com_rad_tmp[i] += M_PI*2.0;
      while(steer_pos_com_rad_tmp[i] < 0.0){
        steer_pos_com_rad_tmp[i] += M_PI*2.0;
      }
    }
  }

  //1番2番の速度を反転
  ctl_v_com[1] = -1.0*ctl_v_com[1];
  ctl_v_com[2] = -1.0*ctl_v_com[2]; 
  



  if(fabs(ctl_v_com[F_L]) > motor_param[MAX_SPEED] || fabs(ctl_v_com[F_R]) > motor_param[MAX_SPEED] || fabs(ctl_v_com[R_L]) > motor_param[MAX_SPEED] || fabs(ctl_v_com[R_R]) > motor_param[MAX_SPEED]){
    double v_com_max = fabs(ctl_v_com[0]);
    int i;
    for(i = 1; i < 4; i++){
      if(v_com_max < fabs(ctl_v_com[i])){
        v_com_max = fabs(ctl_v_com[i]);
      }
    }

    for(i = 0; i < 4; i++){
       ctl_v_com[i] /= (v_com_max/motor_param[MAX_SPEED]);
    }
  }

  /*
  for(i = 0; i < 4; i++){
    steer_pos_now[i] = getSteerRad(i);
  }
  */

  //静止状態への移行の場合、速度が0.0m/s以上の場合は、ステアリング旋回を行わない
  if(ctl_v_com[F_R] == 0.0 && ctl_v_com[F_L] == 0.0 && ctl_v_com[R_L] == 0.0 && ctl_v_com[R_R] == 0.0 && fabs(avr_v[F_R]) > 0.0 && fabs(avr_v[F_L]) > 0.0 && fabs(avr_v[R_L]) > 0.0 && fabs(avr_v[R_R]) > 0.0){
    return;
  }

  //ホイールモータ保護のため、1輪でも現在の角度と指示角度に40度以上の差があった場合、ホイールモータの速度指令値を0にする
  for(i = 0; i < 4; i++){
    if(fabs(steer_pos_now[i] - steer_pos_com_rad_tmp[i]) > DEG_RAD(40.0)){
      ctl_v_com[F_R] = 0.0;
      ctl_v_com[F_L] = 0.0;
      ctl_v_com[R_L] = 0.0;
      ctl_v_com[R_R] = 0.0;
    }
  }

  //printf("%lf | %lf\n", steer_pos_now[0] - steer_pos_com_rad_tmp[0], ctl_v_com[F_R]);


  //タイヤ保護のため、現在の速度が0.1m/s以上の場合、20度以上のステアリング旋回を行わない
  for(i = 0; i < 4; i++){
    if(fabs(avr_v[i]) >= 0.1 && fabs(steer_pos_com_rad_tmp[i] - steer_pos_com_rad[i]) >= DEG_RAD(40.0)){
      
    }else{
      steer_pos_com_rad[i] = steer_pos_com_rad_tmp[i];
    }
  }

}

//メモリマップ記載の左右輪の速度値をそれぞれの目標速度として設定する
void memWheelSPCom2CtlV(){

  setMortorParam2Std();

  ctl_v_com[F_L] = -1.0*wheel.s16Map(MS16_S_LWS)/1000.0;
  ctl_v_com[F_R] = wheel.s16Map(MS16_S_RWS)/1000.0;
  ctl_v_com[R_L] = -1.0*wheel.s16Map(MS16_S_LWS)/1000.0;
  ctl_v_com[R_R] = wheel.s16Map(MS16_S_RWS)/1000.0;

}


//コントローラからの速度指令値に対して加速度制限を実施し、PID制御器への速度指令値を算出する
void ctl2Vcom(){

  //各車輪のv_comとctl_v_comの差を求める
  int i;
  double v_diff[4];
  for(i = 0; i < 4; i++){
    v_diff[i] = ctl_v_com[i] - v_com[i];
  }

  //現在のv_comとctl_v_comの差が最も大きい車輪を探す
  double max_v_diff = fabs(v_diff[F_L]);
  for(i = 1; i < 4; i++){
    if(fabs(v_diff[i]) > max_v_diff){
      max_v_diff = fabs(v_diff[i]);
    }
  }

  //現在のavr_vとv_comの差を調べる
  double diff_avr_com[4] = {0.0, 0.0, 0.0, 0.0};
  for(i = 0; i < 4; i++){
    diff_avr_com[i] = avr_v[i] - v_com[i];
  }

  //現在のavr_vとv_comの差が最も大きい車輪を探す
  double max_diff_avr_com = fabs(diff_avr_com[0]);
  for(i = 1; i < 4; i++){
    if(fabs(diff_avr_com[i]) > max_diff_avr_com){
      max_diff_avr_com = fabs(diff_avr_com[i]);
    }
  }


  //加速時に現在速度とコマンド速度の差の最大値が一定以上になっていた場合、加速度を制限する
  double acc_limiter = 0.0;
  double limited_acc[4] = {0.0, 0.0, 0.0, 0.0};
  for(i = 0; i < 4; i++){
    if(max_diff_avr_com > 0.2){
      limited_acc[i] = motor_param[MAX_ACC] *(fabs(diff_avr_com[i])/max_diff_avr_com);
      if(limited_acc[i] <= 0.0){
        limited_acc[i] = 0.0;
      }
    }else{
      limited_acc[i] = motor_param[MAX_ACC];
    }
  }

  

  //前回から今回の処理までの経過時間
  static uint32_t prev_micros = micros();
  uint32_t new_micros = micros(); //[μsec]
  double elapsed_time = (new_micros - prev_micros); //[μsec]
  if(elapsed_time < 0){
    elapsed_time = (new_micros + (UINT32_MAX - prev_micros)); //[μsec]
  }

  //v_comとctl_v_comの差が最も大きい車輪の加速度がMAX_ACCとなるように計算する
  double add_v = 0.0;
  if(max_v_diff != 0.0){
    for(i = 0; i < 4; i++){
      add_v = ((motor_param[MAX_ACC]/*limited_acc[i]*/ * (v_diff[i]/max_v_diff)*elapsed_time)/1000000.0);
      if(fabs(v_diff[i]) >= fabs(add_v)){
        v_com[i] = v_com[i] + add_v;
      }else{
        v_com[i] = ctl_v_com[i];
      }      
    }
  }



  for(i = 0; i < 4; i++){
    //ctl_v_com = 0.0 かつ v_com < 0.001 なら v_com = 0.0
    if(ctl_v_com[i] == 0.0 && fabs(v_com[i]) < 0.001){
      v_com[i] = 0.0;
    }
  }

  prev_micros = new_micros;

  if(chkBumper(&wheel)){
    v_com[F_L] = 0.0;
    v_com[F_R] = 0.0;
    v_com[R_L] = 0.0;
    v_com[R_R] = 0.0;
    setO_EN(OFF_OFF);
  }else{
    setO_EN(ON_ON);
  }

}

//モータ位置を取得する
void getEncoderValue(){

  wheel.s32Map(MS32_IM_POS0, iMtrFR.readMtr(MS32_M_POS));
  wheel.s32Map(MS32_IM_POS1, iMtrFL.readMtr(MS32_M_POS));
  wheel.s32Map(MS32_IM_POS2, iMtrRL.readMtr(MS32_M_POS));
  wheel.s32Map(MS32_IM_POS3, iMtrRR.readMtr(MS32_M_POS));

}

//モータ回転速度から移動速度を求める
void getRoverV(){

  //wheel.s32Map(MS32_IM_SPD0, iMtrFR.readMtr(MS32_M_SPD));
  //wheel.s32Map(MS32_IM_SPD1, iMtrFL.readMtr(MS32_M_SPD));
  //wheel.s32Map(MS32_IM_SPD2, iMtrRL.readMtr(MS32_M_SPD));
  //wheel.s32Map(MS32_IM_SPD3, iMtrRR.readMtr(MS32_M_SPD));
  
  enc[F_R] = (int32_t)wheel.s32Map(MS32_IM_POS0);
  enc[F_L] = (int32_t)wheel.s32Map(MS32_IM_POS1);
  enc[R_L] = (int32_t)wheel.s32Map(MS32_IM_POS2);
  enc[R_R] = (int32_t)wheel.s32Map(MS32_IM_POS3);

  //前回から今回の処理までの経過時間
  static uint32_t prev_micros = micros();
  uint32_t new_micros = micros(); //[μsec]
  double elapsed_time = ((double)new_micros - (double)prev_micros); //[μsec]
  if(elapsed_time < 0){
    elapsed_time = ((double)new_micros + (UINT32_MAX - (double)prev_micros)); //[μsec]
  }

  int i;
  for(i = 0; i < 4; i++){
    v_enc[i] = (((double)(enc[i]-old_enc[i])/enc_counts_per_turn)*tire_circumference)/(elapsed_time/1000.0);//[m/s]
    avr_v[i] = v_enc[i];
    old_enc[i] = enc[i];
    wheel.s32Map(MS32_IM_SPD0+(i*4), avr_v[i]*1000);
  }

  prev_micros = new_micros;
  

}



//バンパーor電源スイッチが反応していた場合、速度指令値を0にする
uint16_t chkBumper(Wrc058* memmap){

  //バンパーand電源スイッチの状態確認
  //memmap->readMemmap(MU16_M_DI, 2);
  return memmap->u16Map(MU16_IM_DI);
}

//WDTが発動していたら、速度指令値を0にする
uint8_t chkWDT(Wrc058* memmap){

  //WDTの数値を確認し、0xffffなら1をreturn;
  memmap->readMemmap(MU16_WDT, 2);
  if(memmap->u16Map(MU16_WDT) == 0xffff){
    memmap->u8Map(MU8_M_STTS, memmap->u8Map(MU8_M_STTS) | 0x01);
    return 1;
  }
  memmap->u8Map(MU8_M_STTS, memmap->u8Map(MU8_M_STTS) & 0xFE);
  return 0;
}

uint8_t chkWDTFlag(Wrc058* memmap){
  if(memmap->u8Map(MU8_M_STTS) & 0x01){
    return 1;
  }
  return 0;
}


//std_motor_paramの値をmortor_paramに反映する
void setMortorParam2Std(){

  int i;
  for(i = 0; i < 9; i++){
    motor_param[i] = std_motor_param[i];
  }

  return;
}


int currentFlag = 0;
void isCurrentOver(){

  currentFlag = 0;

  uint16_t v_hold = wheel.u16Map(MU16_P_CREF0);

  //wheel.readMemmap(MS16_IM_CUR0, 8);
  //printf("%x | %x\n", wheel.u16Map(MU16_M_CUR0), wheel.u16Map(MU16_M_CUR1));

  if(wheel.u16Map(MS16_IM_CUR0) > v_hold){
    currentFlag = 1;
  }
  if(wheel.u16Map(MS16_IM_CUR1) > v_hold){
    currentFlag = 1;
  }
  if(wheel.u16Map(MS16_IM_CUR2) > v_hold){
    currentFlag = 1;
  }
  if(wheel.u16Map(MS16_IM_CUR3) > v_hold){
    currentFlag = 1;
  }
  

}

//IWSモータ温度の取得
void getMTemp(){

  wheel.s16Map(MS16_IM_TEMP0, (int16_t)iMtrFL.readMtr(MS16_M_DEG));
  wheel.s16Map(MS16_IM_TEMP1, (int16_t)iMtrFR.readMtr(MS16_M_DEG));
  wheel.s16Map(MS16_IM_TEMP2, (int16_t)iMtrRL.readMtr(MS16_M_DEG));
  wheel.s16Map(MS16_IM_TEMP3, (int16_t)iMtrRR.readMtr(MS16_M_DEG));

}

void getMTempStr(){

  steer.s16Map(MS16_IM_TEMP0, (int16_t)iMtrSFL.readMtr(MS16_M_DEG));
  steer.s16Map(MS16_IM_TEMP1, (int16_t)iMtrSFR.readMtr(MS16_M_DEG));
  steer.s16Map(MS16_IM_TEMP2, (int16_t)iMtrSRL.readMtr(MS16_M_DEG));
  steer.s16Map(MS16_IM_TEMP3, (int16_t)iMtrSRR.readMtr(MS16_M_DEG));

}

//IWSモータ電流の取得
void getMCur(){

  wheel.s16Map(MS16_IM_CUR0, (int16_t)iMtrFL.readMtr(MS16_M_CUR));
  wheel.s16Map(MS16_IM_CUR1, (int16_t)iMtrFR.readMtr(MS16_M_CUR));
  wheel.s16Map(MS16_IM_CUR2, (int16_t)iMtrRL.readMtr(MS16_M_CUR));
  wheel.s16Map(MS16_IM_CUR3, (int16_t)iMtrRR.readMtr(MS16_M_CUR));

}

void getMCurStr(){

  steer.s16Map(MS16_IM_CUR0, (int16_t)iMtrSFL.readMtr(MS16_M_CUR));
  steer.s16Map(MS16_IM_CUR1, (int16_t)iMtrSFR.readMtr(MS16_M_CUR));
  steer.s16Map(MS16_IM_CUR2, (int16_t)iMtrSRL.readMtr(MS16_M_CUR));
  steer.s16Map(MS16_IM_CUR3, (int16_t)iMtrSRR.readMtr(MS16_M_CUR));

}

//IWSモータゲインの適用
void updIMGain(){

  if(wheel.writeFlag[MU16_FB_VP0]){
    wheel.writeFlag[MU16_FB_VP0] = 0x00;
    iMtrFR.writeMtr(MU16_FB_SP, 2, &wheel.memMap[MU16_FB_VP0]);
  }
  if(wheel.writeFlag[MU16_FB_VP1]){
    wheel.writeFlag[MU16_FB_VP1] = 0x00;
    iMtrFL.writeMtr(MU16_FB_SP, 2, &wheel.memMap[MU16_FB_VP1]);
  }
  if(wheel.writeFlag[MU16_FB_VP2]){
    wheel.writeFlag[MU16_FB_VP2] = 0x00;
    iMtrRL.writeMtr(MU16_FB_SP, 2, &wheel.memMap[MU16_FB_VP2]);
  }
  if(wheel.writeFlag[MU16_FB_VP3]){
    wheel.writeFlag[MU16_FB_VP3] = 0x00;
    iMtrRR.writeMtr(MU16_FB_SP, 2, &wheel.memMap[MU16_FB_VP3]);
  }

  if(wheel.writeFlag[MU16_FB_VI0]){
    wheel.writeFlag[MU16_FB_VI0] = 0x00;
    iMtrFR.writeMtr(MU16_FB_SI, 2, &wheel.memMap[MU16_FB_VI0]);
  }
  if(wheel.writeFlag[MU16_FB_VI1]){
    wheel.writeFlag[MU16_FB_VI1] = 0x00;
    iMtrFL.writeMtr(MU16_FB_SI, 2, &wheel.memMap[MU16_FB_VI1]);
  }
  if(wheel.writeFlag[MU16_FB_VI2]){
    wheel.writeFlag[MU16_FB_VI2] = 0x00;
    iMtrRL.writeMtr(MU16_FB_SI, 2, &wheel.memMap[MU16_FB_VI2]);
  }
  if(wheel.writeFlag[MU16_FB_VI3]){
    wheel.writeFlag[MU16_FB_VI3] = 0x00;
    iMtrRR.writeMtr(MU16_FB_SI, 2, &wheel.memMap[MU16_FB_VI3]);
  }

  if(wheel.writeFlag[MS16_FB_PP0]){
    wheel.writeFlag[MS16_FB_PP0] = 0x00;
    iMtrFR.writeMtr(MS16_FB_PP, 2, &wheel.memMap[MS16_FB_PP0]);
  }
  if(wheel.writeFlag[MS16_FB_PP1]){
    wheel.writeFlag[MS16_FB_PP1] = 0x00;
    iMtrFL.writeMtr(MS16_FB_PP, 2, &wheel.memMap[MS16_FB_PP1]);
  }
  if(wheel.writeFlag[MS16_FB_PP2]){
    wheel.writeFlag[MS16_FB_PP2] = 0x00;
    iMtrRL.writeMtr(MS16_FB_PP, 2, &wheel.memMap[MS16_FB_PP2]);
  }
  if(wheel.writeFlag[MS16_FB_PP3]){
    wheel.writeFlag[MS16_FB_PP3] = 0x00;
    iMtrRR.writeMtr(MS16_FB_PP, 2, &wheel.memMap[MS16_FB_PP3]);
  }

}

//速度ベースでのIWSモータゲインの設定
void setIMGain(){
  uint16_t kp = 0x0040;
  uint16_t ki_z = 0x0000;
  uint16_t ki_n = 0x0002;

  if(v_com[F_R] == 0.0){
    if(wheel.u16Map(MU16_FB_VP0) != kp){
      wheel.u16Map(MU16_FB_VP0, kp);
    }
    if(wheel.u16Map(MU16_FB_VI0) != ki_z){
      wheel.u16Map(MU16_FB_VI0, ki_z);
    }
    
    //wheel.u16Map(MS16_FB_PP0, 0x0000);
  }else{
    if(wheel.u16Map(MU16_FB_VP0) != kp){
      wheel.u16Map(MU16_FB_VP0, kp);
    }
    if(wheel.u16Map(MU16_FB_VI0) != ki_n){
      wheel.u16Map(MU16_FB_VI0, ki_n);
    }
    //wheel.u16Map(MS16_FB_PP0, 0x0000);
  }

  if(v_com[F_L] == 0.0){
    if(wheel.u16Map(MU16_FB_VP1) != kp){
      wheel.u16Map(MU16_FB_VP1, kp);
    }
    if(wheel.u16Map(MU16_FB_VI1) != ki_z){
      wheel.u16Map(MU16_FB_VI1, ki_z);
    }
    //wheel.u16Map(MS16_FB_PP1, 0x0000);
  }else{
    if(wheel.u16Map(MU16_FB_VP1) != kp){
      wheel.u16Map(MU16_FB_VP1, kp);
    }
    if(wheel.u16Map(MU16_FB_VI1) != ki_n){
      wheel.u16Map(MU16_FB_VI1, ki_n);
    }
    //wheel.u16Map(MS16_FB_PP1, 0x0000);
  }

  if(v_com[R_L] == 0.0){
    if(wheel.u16Map(MU16_FB_VP2) != kp){
      wheel.u16Map(MU16_FB_VP2, kp);
    }
    if(wheel.u16Map(MU16_FB_VI2) != ki_z){
      wheel.u16Map(MU16_FB_VI2, ki_z);
    }
    //wheel.u16Map(MS16_FB_PP0, 0x0000);
  }else{
    if(wheel.u16Map(MU16_FB_VP2) != kp){
      wheel.u16Map(MU16_FB_VP2, kp);
    }
    if(wheel.u16Map(MU16_FB_VI2) != ki_n){
      wheel.u16Map(MU16_FB_VI2, ki_n);
    }
    //wheel.u16Map(MS16_FB_PP0, 0x0000);
  }

  if(v_com[R_R] == 0.0){
    if(wheel.u16Map(MU16_FB_VP3) != kp){
      wheel.u16Map(MU16_FB_VP3, kp);
    }
    if(wheel.u16Map(MU16_FB_VI3) != ki_z){
      wheel.u16Map(MU16_FB_VI3, ki_z);
    }
    //wheel.u16Map(MS16_FB_PP1, 0x0000);
  }else{
    if(wheel.u16Map(MU16_FB_VP3) != kp){
      wheel.u16Map(MU16_FB_VP3, kp);
    }
    if(wheel.u16Map(MU16_FB_VI3) != ki_n){
      wheel.u16Map(MU16_FB_VI3, ki_n);
    }
    //wheel.u16Map(MS16_FB_PP1, 0x0000);
  }

}
