/**
 * モータ出力制御用ライブラリ
 * ROS, V-CON用
 */
#ifndef WRC058_MOTOR_H
#define WRC058_MOTOR_H

#include "vs_wrc058_memmap.h"
#include "vs_wrc058_iws_motor.h"

#define MODE_SPD      1
#define MODE_POS      0

#define RUN_STOP        0
#define RUN_TURN        1
#define RUN_VERTICAL    2
#define RUN_HORIZONTAL  3

#define OFF_OFF    0
#define ON_OFF     1
#define OFF_ON     2
#define ON_ON      3
#define ENC_RESET  0x0c

#define KP_NORMAL  0x03000300   //STM32の比例制御ゲイン
#define KP_ZERO    0x00000000

#define M_L        0 
#define M_R        1  

#define DEG_RAD(x) x*M_PI/180.0     //角度 -> ラジアン
#define RADS_RPM(x) x*30.0/M_PI     //(ステアリングモータの)[rad/s] -> [rpm]
#define SRADS_MRPM(x) x*71.25/M_PI  //(ステアリング軸の)[rad/s] -> (ステアリングモータの)[rpm]
 

//const double ROVER_D = 0.284/2.0;//0.284/2.0;//0.350/2.0;//
//const double TIRE_CIRCUMFERENCE = 477.5;
//const double ENC_COUNTS_PER_TURN = 910.0;//910.0;//998.4;//
//const double ENC_PER_MM = ENC_COUNTS_PER_TURN/TIRE_CIRCUMFERENCE; //単位距離当たりのエンコーダ値

//走行系のパラメータ。関数pidControl()で走行する際にのみ作用します。
extern float std_motor_param[]; //ユーザ設定の初期値
extern float motor_param[];     //発揮値


enum AllMotorList{
  S_F_R = 0,
  S_F_L = 1,
  S_R_L = 2,
  S_R_R = 3,
  W_F_R = 4,
  W_F_L = 5,
  W_R_L = 6,
  W_R_R = 7
};

enum WheelMotorList{
  F_R = 0,
  F_L = 1,
  R_L = 2,
  R_R = 3
};


enum IndexOfMotorParam{
  MAX_SPEED = 0,
  MAX_RAD   = 1,
  K_V2MP    = 2,
  K_P       = 3,
  K_I       = 4,
  K_D       = 5,
  PAD_DEAD  = 6,
  PAD_MAX   = 7,
  MAX_ACC   = 8,
  MAX_STR_S = 9,
  MAX_STR_A = 10,
  STR_K_P   = 11
};

extern uint8_t run_mode;

extern double rover_d;  //車輪間距離（車幅方向） MECA2.1 -> 0.266/2.0 MEGAG120 -> 0.400/2.0
extern double rover_hb; //車輪間距離（ホイールベース方向）MECA2.1 -> 0.230/2.0  MECAG120 -> 0.380/2.0
extern double tire_circumference;   //タイヤ直径　MECA2.1 -> 477.52 MEGAG120 -> 628.32
extern double enc_counts_per_turn;  //タイヤ1回転当たりのエンコーダカウント値 MECA2.1 -> 910.0 MEGAG120 -> 998.4
extern double enc_per_mm; //単位距離当たりのエンコーダ値
extern uint8_t rover_type;

extern double  ctl_v_com[4];
extern double v_com[4];
extern int16_t m_com[4];
extern int16_t prev_m[4];
extern double v_enc[4];
extern double v_diff[4];
extern double avr_v[4];
extern double prev_v_diff[4];

extern int32_t enc[4];
extern int32_t old_enc[4];

extern double  steer_pos_now[4];
extern double  steer_center[4];
extern uint8_t init_step;

extern int currentFlag;

extern uint32_t pid_time;

void setBodyDetail(int type);
int setO_EN(int data);
int setO_EN(int data, Wrc058* memmap);
int getO_EN();
int getO_EN(Wrc058* memmap);
void updO_EN();
void updO_EN_S();
int setCtrlMode(int ctrl_mode);
int setCtrlMode(int ctrl_mode, Wrc058* memmap);
int getCtrlMode();
int getCtrlMode(Wrc058* memmap);
void updIWSMode();
void steeringMotorEnable(IwsInwheelMotor* inst_iMtr);
void steeringMotorDisable(IwsInwheelMotor* inst_iMtr);
void resetSteeringMotorEnc(IwsInwheelMotor* inst_iMtr);
bool isInitSteerFin();
int initSteer();
void getSteerEncoder(uint8_t motor_no);
double getSteerRad(uint8_t motor_no);
void moveSteerPosV();
void aStick2V();
void memCom2V();
void memWheelSPCom2CtlV();
void ctl2Vcom();
void getEncoderValue();
void getRoverV();
uint16_t chkBumper(Wrc058* memmap);
uint8_t chkWDT(Wrc058* memmap);
uint8_t chkWDTFlag(Wrc058* memmap);
void setMortorParam2Std();
void isCurrentOver();
void getMTemp();
void getMTempStr();
void getMCur();
void getMCurStr();
void updIMGain();
void setIMGain();

#endif
