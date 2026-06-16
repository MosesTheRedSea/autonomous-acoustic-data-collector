#include "vs_wrc058_fwdsrover.h"
#include "vs_wrc058_memmap.h"
#include "vs_wrc058_serial.h"
#include "vs_wrc058_motor.h"
#include "vs_wrc058_iws_motor.h"
#include "vs_wrc058_ble_pad.h"

//LED制御のための定義
#define LEDC_CHANNEL_0     0
#define LEDC_TIMER_8_BIT   8
#define LEDC_BASE_FREQ     5000
#define LED_PIN            4


//PAD入力内容の判定
void chkPadInput(){

uint32_t button_on = 0;
  if(checkBTN(S_R1) || checkBTN(S_L1) || checkBTN(S_L2) || checkBTN(S_R2)){   //S_R2を除くいずれかのショルダーボタンが押されていたら、アナログスティックの入力に応じて動作する
    if(checkBTN(S_R1) && checkBTN(S_R2)){                                     //押されているボタンに応じて最高速度を設定する
      run_mode = RUN_VERTICAL;
      motor_param[MAX_SPEED] = std_motor_param[MAX_SPEED];
      motor_param[MAX_RAD]   = std_motor_param[MAX_RAD];
    }else if(checkBTN(S_L1) && checkBTN(S_R2)){
      run_mode = RUN_HORIZONTAL;
      motor_param[MAX_SPEED] = std_motor_param[MAX_SPEED];
      motor_param[MAX_RAD]   = std_motor_param[MAX_RAD];
    }else if(checkBTN(S_L2) && checkBTN(S_R2)){
      run_mode = RUN_TURN;
      motor_param[MAX_SPEED] = std_motor_param[MAX_SPEED];
      motor_param[MAX_RAD]   = std_motor_param[MAX_RAD];
    }else if(checkBTN(S_R2)){
      run_mode = RUN_STOP;
      motor_param[MAX_SPEED] = 0.0;
      motor_param[MAX_RAD]   = 0.0;
    }else if(checkBTN(S_R1)){
      run_mode = RUN_VERTICAL;
      motor_param[MAX_SPEED] = 0.8;
      motor_param[MAX_RAD]   = 2.51;
    }else if(checkBTN(S_L1)){
      run_mode = RUN_HORIZONTAL;
      motor_param[MAX_SPEED] = 0.8;
      motor_param[MAX_RAD]   = 2.51;
    }else if(checkBTN(S_L2)){
      run_mode = RUN_TURN;
      motor_param[MAX_SPEED] = 0.8;
      motor_param[MAX_RAD]   = 2.51;
    }
    //setCtrlMode(MODE_POS);                                                    //ctrlモードをPWMにして、
    //setO_EN(ON_ON);                                                  //モータ出力をONにし、
    //aStick2V();                                                               //アナログスティックの傾きを速度指令値に変換する
      
  }else{
    run_mode = RUN_STOP;
    setO_EN(ON_ON, &wheel);
    setMortorParam2Std();                                                     //モータ制御パラメータをスタンダード値に設定し、
    //aStick2V();                                                               //アナログスティックの傾きを速度指令値に変換する
    ctl_v_com[F_R] = 0.0;                                                         //速度指令値を0にする
    ctl_v_com[F_L] = 0.0;
    ctl_v_com[R_L] = 0.0;
    ctl_v_com[R_R] = 0.0;
  }

  wheel.s16Map(MS16_S_XS, 0);
  wheel.s16Map(MS16_S_YS, 0);
  wheel.s16Map(MS16_S_ZS, 0);

  if(checkBTN(CROSS_U)){  //十字ボタン上で低速前進
    button_on = 1;
    //setCtrlMode(MODE_POS);
    //setO_EN(ON_ON);
    wheel.s16Map(MS16_S_XS, wheel.s16Map(MS16_S_XS) + 300);
    wheel.s16Map(MS16_S_YS, wheel.s16Map(MS16_S_YS) +   0);
    wheel.s16Map(MS16_S_ZS, wheel.s16Map(MS16_S_ZS) +   0);
 
  }
  if(checkBTN(CROSS_D)){  //十字ボタン下で低速後進
    button_on = 1;
    wheel.s16Map(MS16_S_XS, wheel.s16Map(MS16_S_XS) - 300);
    wheel.s16Map(MS16_S_YS, wheel.s16Map(MS16_S_YS) +   0);
    wheel.s16Map(MS16_S_ZS, wheel.s16Map(MS16_S_ZS) +   0);

  }
  if(checkBTN(CROSS_L)){  //十字ボタン左で低速左進
    button_on = 1;
    wheel.s16Map(MS16_S_XS, wheel.s16Map(MS16_S_XS) +   0);
    wheel.s16Map(MS16_S_YS, wheel.s16Map(MS16_S_YS) + 300);
    wheel.s16Map(MS16_S_ZS, wheel.s16Map(MS16_S_ZS) +   0);

  }
  if(checkBTN(CROSS_R)){  //十字ボタン右で低速右進
    button_on = 1;
    wheel.s16Map(MS16_S_XS, wheel.s16Map(MS16_S_XS) +   0);
    wheel.s16Map(MS16_S_YS, wheel.s16Map(MS16_S_YS) - 300);
    wheel.s16Map(MS16_S_ZS, wheel.s16Map(MS16_S_ZS) +   0);

  }

  if(checkBTN(TRIANGLE)){  //△ボタンで増速（or微速前進）
    button_on = 1;
    if(wheel.s16Map(MS16_S_XS) > 0){
      wheel.s16Map(MS16_S_XS, wheel.s16Map(MS16_S_XS) + 100);
    }
    if(wheel.s16Map(MS16_S_YS) > 0){
      wheel.s16Map(MS16_S_YS, wheel.s16Map(MS16_S_YS) + 100);
    }
    if(wheel.s16Map(MS16_S_XS) < 0){
      wheel.s16Map(MS16_S_XS, wheel.s16Map(MS16_S_XS) - 100);
    }
    if(wheel.s16Map(MS16_S_YS) < 0){
      wheel.s16Map(MS16_S_YS, wheel.s16Map(MS16_S_YS) - 100);
    }
    if(wheel.s16Map(MS16_S_XS) == 0 && wheel.s16Map(MS16_S_YS) == 0){
      wheel.s16Map(MS16_S_XS, wheel.s16Map(MS16_S_XS) + 100);
    }
    
    wheel.s16Map(MS16_S_ZS, wheel.s16Map(MS16_S_ZS) +   0);

  } 
  if(checkBTN(CROSS)){  //×ボタンで微速後進
    button_on = 1;
    if(wheel.s16Map(MS16_S_XS) > 0){
      wheel.s16Map(MS16_S_XS, wheel.s16Map(MS16_S_XS) - 100);
    }
    if(wheel.s16Map(MS16_S_YS) > 0){
      wheel.s16Map(MS16_S_YS, wheel.s16Map(MS16_S_YS) - 100);
    }
    if(wheel.s16Map(MS16_S_XS) < 0){
      wheel.s16Map(MS16_S_XS, wheel.s16Map(MS16_S_XS) + 100);
    }
    if(wheel.s16Map(MS16_S_YS) < 0){
      wheel.s16Map(MS16_S_YS, wheel.s16Map(MS16_S_YS) + 100);
    }
    if(wheel.s16Map(MS16_S_XS) == 0 && wheel.s16Map(MS16_S_YS) == 0){
      wheel.s16Map(MS16_S_XS, wheel.s16Map(MS16_S_XS) - 100);
    }

    wheel.s16Map(MS16_S_ZS, wheel.s16Map(MS16_S_ZS) +   0);

  }
  if(checkBTN(SQUARE)){  //□ボタンで微速左回転
    button_on = 1;
    wheel.s16Map(MS16_S_XS, wheel.s16Map(MS16_S_XS) +   0);
    wheel.s16Map(MS16_S_YS, wheel.s16Map(MS16_S_YS) +   0);
    wheel.s16Map(MS16_S_ZS, wheel.s16Map(MS16_S_ZS) + 523);

  }
  if(checkBTN(CIRCLE)){  //〇ボタンで微速右回転
    button_on = 1;
    wheel.s16Map(MS16_S_XS, wheel.s16Map(MS16_S_XS) +   0);
    wheel.s16Map(MS16_S_YS, wheel.s16Map(MS16_S_YS) +   0);
    wheel.s16Map(MS16_S_ZS, wheel.s16Map(MS16_S_ZS) - 523);

  }

  if(button_on){
    memCom2V();
  }else{
    aStick2V();
  }
}


//ESP32電源の電子スイッチを操作する
void setLPOW_EN(bool lpow){
  pinMode(2, OUTPUT);

  digitalWrite(2, lpow);
}

//ESP32電源の電子スイッチをONに
void setLPOW_ON(){
  setLPOW_EN(true);
}

//ESP32電源の電子スイッチをOFFに
void setLPOW_OFF(){
  setLPOW_EN(false);
}


//モータ電源の電子スイッチを操作する
void setMPOW_EN(bool mpow){
  if(mpow){
    wheel.u8Map(MU8_O_EN, ON_ON);
  }else{
    wheel.u8Map(MU8_O_EN, OFF_OFF);
  }
}

//モータ電源の電子スイッチをONに
void setMPOW_ON(){
  setMPOW_EN(true);
}

//モータ電源の電子スイッチをOFFに
void setMPOW_OFF(){
  setMPOW_EN(false);
}


//RS485のDEを操作する
void setESP485_DE(bool de){
  pinMode(13, OUTPUT);

  digitalWrite(13, de);
}

//モータ電源の電子スイッチをONに
void setESP485_DE_ON(){
  setESP485_DE(true);
}

//モータ電源の電子スイッチをOFFに
void setESP485_DE_OFF(){
  setESP485_DE(false);
}

/*******************************************
 * 電源ボタン関連
 */
//電源ボタンピンの設定
void initP_BTN(){
  pinMode(15, INPUT);
}

//電源ボタンが2秒以上押されていたら電源OFF
void chkP_BTN(){
  static uint16_t pBtnCount = 0;

  if(digitalRead(15)){
    pBtnCount++;
  }else{
    pBtnCount = 0;
  }

  if(pBtnCount > 200){  //10ms周期で処理　200 = 2000ms
    wheel.u8Map(MU8_CR0, wheel.u8Map(MU8_CR0) | 0x02);
    setLPOW_OFF();
  }

  //Serial.println(pBtnCount);

}

/*******************************************
 * バッテリー電圧関連
 */
//バッテリー電圧測定セットアップ
void initV_IN(){
  pinMode(34, INPUT);
}

//バッテリー電圧の測定
int getV_IN(){

  wheel.u16Map(MU16_M_VI, analogRead(34)+200);   //補正込み

  return wheel.u16Map(MU16_M_VI);
}

//電圧チェック：設定値を3秒以上連続して下回った場合は電源OFF
void chkVLevel(){
  static uint16_t lowLevelCount = 0;

  if(wheel.u16Map(MU16_M_VI) < wheel.u16Map(MU16_SD_VI)){
    lowLevelCount++;
  }else{
    lowLevelCount = 0;
  }

  if(lowLevelCount > 300){
    wheel.u8Map(MU8_CR0, wheel.u8Map(MU8_CR0) | 0x02);
    LED(0);
    lowLevelCount = 300;
    //setLPOW_OFF();
    //setMPOW_OFF();
  }

}

/*******************************************
 * メモリマップによる電源OFF
 */
//MU16_POWOFF_Tに値が書き込まれた場合、そのms後にシャットダウンする
int chkPowOffT(){
  int powOffTime = 0;
  uint8_t witchMemMap = 0;

  if(wheel.u16Map(MU16_POWOFF_T) != 0){
    powOffTime = wheel.u16Map(MU16_POWOFF_T);
    witchMemMap = S_MAIN;
  }else{
    return -1;
  }

  powOffTime -= 10;   //制御周期10ms
  Serial.println(powOffTime);

  if(powOffTime <= 0){
    wheel.u8Map(MU8_CR0, wheel.u8Map(MU8_CR0) | 0x02);
    LED(0);
    setMPOW_OFF();
    //setLPOW_OFF();

    return 0;
  }

  wheel.u16Map(MU16_POWOFF_T, powOffTime);

  return powOffTime;

}

/*******************************************
 * タイマー割り込み設定
 */
hw_timer_t *interruptTimer = NULL;
hw_timer_t *interruptTimerS = NULL; 
volatile bool isInterrupt = false;
volatile bool isInterrupt_s = false;

//volatile SemaphoreHandle_t timerSemaphore;
//volatile SemaphoreHandle_t timerSemaphoreS;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMuxS = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux);
  isInterrupt = true;
  portEXIT_CRITICAL_ISR(&timerMux);
  //xSemaphoreGiveFromISR(timerSemaphore, NULL);

}

void IRAM_ATTR onTimerS(){
  portENTER_CRITICAL_ISR(&timerMuxS);
  isInterrupt_s = true;
  portEXIT_CRITICAL_ISR(&timerMuxS);
  //xSemaphoreGiveFromISR(timerSemaphoreS, NULL);
}

void setupInterruptTimer(){
  //セマフォを設定
  //timerSemaphore = xSemaphoreCreateBinary();

  //4つ中1つ目と2つ目のタイマーを使用する（カウントは0から）
  //dividerを80に設定
  interruptTimer  = timerBegin(0, 80, true);
  interruptTimerS = timerBegin(1, 80, true);

  //割り込みで発砲する関数にonTimer()を設定
  timerAttachInterrupt(interruptTimer, &onTimer, true);
  timerAttachInterrupt(interruptTimerS, &onTimerS, true);

  //割り込み周期を設定(単位はmicroSec)
  //繰り返し割り込みを許可（第３引数）
  timerAlarmWrite(interruptTimer, 50000, true);
  timerAlarmWrite(interruptTimerS, 50000, true);

  //割り込み判定開始
  timerAlarmEnable(interruptTimer);
  timerAlarmEnable(interruptTimerS);

}

/*******************************************
 * LED関連
 */
//LEDのセットアップ
void ledInit(){
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);

  return;
}

//LED制御関数
void LED(int cmd){
  if(cmd){
    //LEDを点ける
    ledcWrite(LEDC_CHANNEL_0, 255);
  }else{
    //LEDを消す
    ledcWrite(LEDC_CHANNEL_0, 0);
  }

  return;
}

//メモリマップからのLED操作(MU8_CR0 b1 highなら消灯,lowならフリー)
void chkLEDFlag(){

  if(wheel.memMap[MU8_CR0] & 0x02){
    LED(0);
  }else{
    LED(1);
  }

  return;
}

/*******************************************
 * デジタル入力関連
 */
//バンパー入力のセットアップ
const uint8_t DIN[2][8] = {{25,32,19,23,5,18,33,35},{25,32,14,/*15*/13,36,39,35,33}};

void setDIN(uint8_t rover_type){
  int i;
  if(rover_type == FWDSROVER_X40A){
    for(i = 0; i < 8; i++){
      pinMode(DIN[1][i],INPUT_PULLUP);
    }
    pinMode(DIN[1][6],INPUT);
  }else{
    for(i = 0; i < 7; i++){
      pinMode(DIN[0][i],INPUT_PULLUP);
    }
    pinMode(DIN[0][7],INPUT);
  }
  

}

//バンパー入力の読み込み
void chkDIN(uint8_t rover_type){

  int i;
  if(rover_type == FWDSROVER_X40A){
    for(i = 0; i < 8; i++){
      if(digitalRead(DIN[1][i])){
        wheel.u16Map(MU16_IM_DI, wheel.u16Map(MU16_IM_DI) & ~(0x0001 << i));
      }else{
        wheel.u16Map(MU16_IM_DI, wheel.u16Map(MU16_IM_DI) | (0x0001 << i));
      }
    }
  }else{
    for(i = 0; i < 8; i++){
      if(digitalRead(DIN[0][i])){
        wheel.u16Map(MU16_IM_DI, wheel.u16Map(MU16_IM_DI) & ~(0x0001 << i));
      }else{
        wheel.u16Map(MU16_IM_DI, wheel.u16Map(MU16_IM_DI) | (0x0001 << i));
      }
    }
  }

  return;
}

/*******************************************
 * 時間管理
 */
void setTimeNow(){
  wheel.u32Map(MU32_TRIPTIME, millis());

  return;
}