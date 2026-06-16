/**
 * スケッチ名：fwdsrover_x40a_common
 * バージョン：30.00
 * 概要：
 * 　4WDSローバーを外部のデバイスから制御する際に汎用的に使用可能なスケッチです。
 * 　パッド->USBシリアル=ROS(USBシリアル)->ROS(Wi-Fi)->HTTP　の優先順位で操作コマンドを受理します。
 * 　
 *   ROS使用時にROSとの接続がない場合、自動的に停止します。ただし、付属ゲームパッドによる操作は有効です。
 * 
 */
#define ROVER_TYPE FWDSROVER_X40A //削除するとビルドエラーになります。


/*******************************************************************
 * WRC_WIRELESS_MODEに使用する規格を定義してください。
 * 
 * 〇WiFi使用時
 * WRC_WIRELESS_MODE を USE_WIFI で定義してください。
 * また、アクセスポイントのSSIDとパスワードも設定してください(L67,68)。
 * 
 * 
 * 〇無線通信を使用しないとき
 * WRC021_WIRELESS_MODE を WIRELESS_OFF で定義してください。
 */ 
#define WIRELESS_OFF 0
#define USE_WIFI     1
#define WRC_WIRELESS_MODE   WIRELESS_OFF

/*******************************************************************
 * 本製品はROSから制御することが可能です。
 * WiFi接続と有線シリアル接続を選ぶことができます。両方同時には使えません。
 * WRC_ROS_SERIAL_MODEに使用する規格を定義してください。
 * 
 * 〇WiFi接続時
 * WRC_ROS_SERIAL_MODE を MODE_WIFI で定義してください。
 * また、WRC021_WIRELESS_MODE を USE_WIFI で定義してください。
 * 
 * 〇有線シリアル使用時
 * WRC_ROS_SERIAL_MODE を MODE_SERIAL で定義してください。
 * 
 * 〇ROSを使用しないとき
 * WRC_ROS_SERIAL_MODE を MODE_OFF で定義してください。
 */
#define MODE_OFF    0
#define MODE_WIFI   1
#define MODE_SERIAL 2
#define WRC_ROS_SERIAL_MODE MODE_SERIAL

/*******************************************************************
 * ウォッチドッグタイマに関する設定
 * 何らかの要因により外部機器との通信が停止した場合、WDTIMEに設定した時間[ms]後にモータを停止します。
 * 0を設定するとESP32によるタイマのクリアは無効化されますが、
 * 外部からメモリマップのMU16_WDTに値をセットすることでクリアすることができます。
 */
#define WDTIME 0

#include <vs_wrc058_fwdsrover.h>
#include <vs_wrc021_ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <WiFi.h>
#include <Arduino.h>
#include <NimBLEDevice.h>


/*******************************************
 * プロトタイプ宣言
 */
void twistCallBack(const geometry_msgs::Twist& );
void setRoverOdo();
void setRoverSensor();
void LED(int cmd);


/*******************************************
 * WiFi関連の設定
 */
const char* ui_path = "/index.html";  //HTMLコントローラのパス
const char* ssid     = "";            //WiFi使用時に接続するアクセスポイントのSSIDを設定してください ※2.4GHzのみ対応
const char* password = "";            //WiFi使用時に接続するSSIDのパスワードを設定してください

/*******************************************
 * ROS接続設定
 */
IPAddress ROSserver(192,168,1,1);        //rosserial socket serverのIPアドレスを設定してください。
const uint16_t serverPort = 11411;       //rosserial socket serverのポートを設定してください。

/*******************************************
 *  ROS用の各種定義
 */
ros::NodeHandle nh;     //ノードハンドル

std_msgs::Int16MultiArray rover_sensor;
geometry_msgs::Twist rover_odo;
ros::Publisher pub_sensor("rover_sensor", &rover_sensor);
ros::Publisher pub_odo("rover_odo", &rover_odo);
ros::Subscriber<geometry_msgs::Twist> sub_twist("rover_twist", &twistCallBack);

int publishInterval = millis();   //ROSパブリッシュのタイミング制御
int spinOnceInterval = millis();  //ROS spinOnceのタイミング制御

const int publish_rate = 20;      //publishする周波数[Hz]

/*******************************************
 * ステアリング通信タスク
 */
TaskHandle_t thp[1];
void ctrlSteer(void *args){
  while(1){
    if(isInterrupt_s){
      portENTER_CRITICAL_ISR(&timerMuxS);
      isInterrupt_s = false;
      portEXIT_CRITICAL_ISR(&timerMuxS);

      if(isInitSteerFin()){
        moveSteerPosV();
      }else{
        initSteer();
      }

      updO_EN_S();
    }

    delay(1);

  }
}


/*******************************************
 * セットアップ
 */
void setup()
{
  Serial.begin(115200);
  Serial.println("start_setup");


  //モータ制御関連パラメータの設定(適切な値を設定してください)
  std_motor_param[MAX_SPEED] = 1.6;     //最大速度[m/s]
  std_motor_param[MAX_RAD]   = 3.14;    //最大旋回速度[rad/s]
  std_motor_param[MAX_ACC]   = 1.2;     //タイヤ回転速度に対する最大加減速度[m/s^2] 
  setMortorParam2Std(); 
  setBodyDetail(ROVER_TYPE);

  //ウォッチドッグタイマの設定
  wdTime = WDTIME;

  //タイマー割り込み設定
  setupInterruptTimer();
  
  //LEDセットアップ
  ledInit();
  LED(1);                         //LEDを点ける

  //バッファのクリア
  clearPersBuf();

  //[ROS]接続モード別設定値の設定
  #if (WRC_ROS_SERIAL_MODE == MODE_WIFI)
    // using ROS over Wi-Fi
    nh.getHardware()->setConnection(ROSserver, serverPort);
  #else
    // using ROS via USB-Serial or do not use ROS
    nh.getHardware()->setBaud(115200); 
    nh.getHardware()->setRead(readMsg4ROS);
  #endif

  //各無線通信機能のセットアップ
  #if (WRC_WIRELESS_MODE == USE_WIFI)
    wifiInit((char* )ssid, (char* )password);
    serverInit((char* )ui_path);
  #endif
  
  

  //[ROS]ros serial初期設定
  nh.initNode();  //MODE_SERIAL or MODE_OFFの場合、この内部でSerail.begin()が行われます

  //[ROS]publish & subscribeの設定
  nh.advertise(pub_sensor);
  nh.advertise(pub_odo);
  nh.subscribe(sub_twist);

  //[ROS]センサメッセージのメモリ領域確保
  rover_sensor.data_length = 2;
  rover_sensor.data = (int16_t *)malloc(sizeof(int16_t)*rover_sensor.data_length);


  delay(10);
  LED(0);  

  
  //メモリマップ初期化
  wheel.memMapClean();
  wheel.initMemmap(18.0);
  steer.memMapClean();
  steer.initMemmap(18.0);

  delay(50);
  LED(1);  

  initP_BTN();
  initV_IN();

  Serial.println("start_init");


  iMtrFR.setDEPin(12);
  iMtrFL.setDEPin(12);
  iMtrRL.setDEPin(12);
  iMtrRR.setDEPin(12);
  SerialIWS.begin(38400, SERIAL_8N1, 26, 27);

  iMtrFR.changeBaud();
  iMtrFL.changeBaud();
  iMtrRL.changeBaud();
  iMtrRR.changeBaud();

  SerialIWS.begin(115200, SERIAL_8N1, 26, 27);

  iMtrSFR.setDEPin(/*13*/15);
  iMtrSFL.setDEPin(15);
  iMtrSRL.setDEPin(15);
  iMtrSRR.setDEPin(15);
  SerialIWS_ST.begin(38400, SERIAL_8N1, 16, 17);

  iMtrSFR.changeBaud();
  iMtrSFL.changeBaud();
  iMtrSRL.changeBaud();
  iMtrSRR.changeBaud();

  SerialIWS_ST.begin(115200, SERIAL_8N1, 16, 17);

  iMtrFL.init(ROVER_TYPE);
  iMtrFR.init(ROVER_TYPE);
  iMtrRL.init(ROVER_TYPE);
  iMtrRR.init(ROVER_TYPE);
  iMtrSFR.init(ROVER_TYPE);
  iMtrSFL.init(ROVER_TYPE);
  iMtrSRL.init(ROVER_TYPE);
  iMtrSRR.init(ROVER_TYPE);

  setDIN(ROVER_TYPE);

  setCtrlMode(MODE_SPD);
  setO_EN(ON_ON, &wheel);

  spiInit();

  xTaskCreatePinnedToCore(ctrlSteer, "ctrlSteer", 8192, NULL, 1, &thp[0], 0);

  LED(0);  


  Serial.println("readey");
  LED(1);                         //LEDを消す

}


/*******************************************
 * メインループ
 */
void loop(){

  LED(1);
  chkVLevel();

  int code = NO_INPUT;    //状態判定のためのコード


  if(Serial.available()){                                                               //シリアル入力があった場合、
    //LED(0);                                                                             //LEDを点け、
    code = SERIAL_ACCES;                                                                //状態コードをSERIAL_ACCESとする。
    if(!persMsgViaSerial()){                                                            //シリアル入力を解釈する。
      //無効なシリアルアクセス                                                            //
      code = NO_INPUT;                                                                  //無効なシリアルアクセスであった場合は状態コードをNO_INPUTとする
    }
    
  }else if(WRC_ROS_SERIAL_MODE == MODE_WIFI && WiFi.status() == WL_CONNECTED){       //Wi-Fi接続でROSを使用し、Wi-Fiアクセスポイントとの接続が確立していて、
    if(millis() - spinOnceInterval > 10){                                               //かつ前回の処理から10ms以上経過していた場合、
      spinOnceInterval = millis();                                                      //
      if(nh.spinOnce() != -1){                                                          //ros_spinOnce()を実行し
        //ROSからの指令値取得                                                             //ROSからの指令値であれば
        //LED(0);               //LEDを点ける                                              //LEDを点灯し
        code = ROS_CTRL;                                                                //状態コードをROS_CTRLとする
      }
    }
  }else{
    if(WiFi.status() == WL_CONNECTED){                                                  //Wi-Fiアクセスポイントとの接続が確立していていれば
      code = checkClient();                                                             //HTTPアクセスの確認および処理を行い
      if(code){                                                                         //有効なアクセスであれば
        //LED(0);               //LEDを点ける                                              //LEDを点灯する
      }
    }
  }

  if(WRC_ROS_SERIAL_MODE == MODE_SERIAL || !rcvMsg4ROS.empty()){                     //rosserialを有線シリアル接続で使用するなら
      spinOnceInterval = millis();                                                      //
      if(nh.spinOnce() == 0){                                                          //spinOnceを実行する
        //ROSからの指令値取得                                                             //ROSからの指令値であれば
        code = ROS_CTRL;                                                                //状態コードをROS_CTRLとする。
      }else{
        //rcvMsg4ROS.clear();
        code = ROS_ERR;
      }
  }


  if(isInterrupt){
    portENTER_CRITICAL_ISR(&timerMux);
    isInterrupt = false;
    portEXIT_CRITICAL_ISR(&timerMux);

    setTimeNow();
    //chkPowOffT();
    chkDIN(ROVER_TYPE);

    getEncoderValue();
    getRoverV();


    //ROS使用時にROSが未接続の場合、速度指令値を0.0[m/s]とする
    if((WRC_ROS_SERIAL_MODE == MODE_SERIAL || WRC_ROS_SERIAL_MODE == MODE_WIFI) && !nh.connected()){
      wheel.s16Map(MS16_S_XS, 0);
      wheel.s16Map(MS16_S_YS, 0);
      wheel.s16Map(MS16_S_ZS, 0);
    }


    if(updatePad() || existsPadInput()){                                                  //PADの入力があった場合、
      LED(0);                                                                             //LEDを点け、                                                                                        
      code = PAD_INPUT;                                                                   //状態コードをPAD_INPUTとし、
      chkPadInput();                                                                      //PADの入力内容を判定する。
    }else{
      LED(1);
      memCom2V();
    }

    ctl2Vcom();

    setIMGain();
    updIMGain();

    if(isInitSteerFin()){
      iMtrFL.sendMTRSpd(iMtrFL.calcTSpd(iMtrFL.calcRPMCom(v_com[F_L])));
      iMtrFR.sendMTRSpd(iMtrFR.calcTSpd(iMtrFR.calcRPMCom(v_com[F_R])));
      iMtrRL.sendMTRSpd(iMtrRL.calcTSpd(iMtrRL.calcRPMCom(v_com[R_L])));
      iMtrRR.sendMTRSpd(iMtrRR.calcTSpd(iMtrRR.calcRPMCom(v_com[R_R])));
    }

    getMCur();

    getV_IN();
    chkWDT(&wheel);

    setO_EN(ON_ON, &wheel);
    updO_EN();
    chkLEDFlag();
    
  }

  //ROSメッセージの送信
  if(WRC_ROS_SERIAL_MODE != MODE_OFF && nh.connected()){
    if(millis() - publishInterval > 1000/publish_rate){
      publishInterval = millis();
      setRoverOdo();
      setRoverSensor();
      pub_odo.publish( &rover_odo);
      pub_sensor.publish( &rover_sensor);
    }
  }

}

/*******************************************
 * ROSメッセージ rover_twist のコールバック
 */ 
void twistCallBack(const geometry_msgs::Twist& msg){
  wheel.s16Map(MS16_S_XS, (int16_t)(msg.linear.x*1000));
  wheel.s16Map(MS16_S_YS, (int16_t)(msg.linear.y*1000));
  wheel.s16Map(MS16_S_ZS, (int16_t)(msg.angular.z*1000));
  setCtrlMode(MODE_SPD);
  setO_EN(ON_ON);
  return;
}

/*******************************************
 * ROSメッセージ rover_odo へのオドメトリ情報の入力
 */
void setRoverOdo(){
  double steer_rad_w[4] = {0.0, 0.0, 0.0, 0.0}; //ロボット座標系から見た各ステアリング角
  steer_rad_w[F_R] = steer_pos_now[F_R] - steer_center[F_R] - (M_PI/4.0);
  steer_rad_w[F_L] = steer_pos_now[F_L] - steer_center[F_R] + (M_PI/4.0);
  steer_rad_w[R_L] = steer_pos_now[R_L] - steer_center[F_R] - (M_PI/4.0);
  steer_rad_w[R_R] = steer_pos_now[R_R] - steer_center[F_R] + (M_PI/4.0);

  int i;
  for(i = 0; i < 4; i++){
    while(steer_rad_w[i] < 0.0){
      steer_rad_w[i] += M_PI*2.0;
    }
  }

  double wheel_vx[4] = {0.0, 0.0, 0.0, 0.0};  //各輪の速度ベクトルのx成分
  double wheel_vy[4] = {0.0, 0.0, 0.0, 0.0};  //各輪の速度ベクトルのy成分
  for(i = 0; i < 4; i++){
    wheel_vx[i] = avr_v[i]*cos(steer_rad_w[i]);
    wheel_vy[i] = -1.0*avr_v[i]*sin(steer_rad_w[i]);
  }

  //F_L/R_Lの速度を反転
  for(i = 1; i < 3; i++){
    wheel_vx[i] = -1.0*wheel_vx[i];
    wheel_vy[i] = -1.0*wheel_vy[i];
  }

  double rover_vx = 0.0;
  double rover_vy = 0.0;
  for(i = 0; i < 4; i++){
    rover_vx += wheel_vx[i];
    rover_vy += wheel_vy[i];
  }
  rover_vx /= 4.0;
  rover_vy /= 4.0;
  
  rover_odo.linear.x = rover_vx;
  rover_odo.linear.y = rover_vy;

  //printf("%lf| %lf| %lf| %lf\n", wheel_vx[F_R], wheel_vx[F_L], wheel_vx[R_L], wheel_vx[R_R]);

  uint8_t run_mode_odo = RUN_TURN;
  if(fabs(steer_pos_now[F_R]-steer_center[F_R]) < 0.01 && fabs(steer_pos_now[F_L]-steer_center[F_L]) < 0.01 && fabs(steer_pos_now[R_L]-steer_center[R_L]) < 0.01 && fabs(steer_pos_now[R_R]-steer_center[R_R]) < 0.01){
    run_mode_odo = RUN_TURN;
  }else if(fabs(rover_vx) >= fabs(rover_vy)){
    run_mode_odo = RUN_VERTICAL;
  }else{
    run_mode_odo = RUN_HORIZONTAL;
  }

  double v_l = 0.0;
  double v_r = 0.0;
  if(run_mode_odo == RUN_TURN){
    rover_odo.angular.z = ((avr_v[F_R] + avr_v[F_L] + avr_v[R_L] + avr_v[R_R])/4.0)/(sqrt(rover_d*rover_d + rover_hb*rover_hb));
    
  }else if(run_mode_odo == RUN_VERTICAL){
    v_l = (avr_v[F_L] + avr_v[R_L])/2.0;
    v_r = (avr_v[F_R] + avr_v[R_R])/2.0;
    rover_odo.angular.z = ((v_r - (-1.0*v_l))/(2.0*rover_d));
  }else if(run_mode_odo == RUN_HORIZONTAL){
    v_l = (avr_v[R_L] + avr_v[R_R])/2.0;
    v_r = (avr_v[F_L] + avr_v[F_R])/2.0;
    rover_odo.angular.z = ((v_r - (-1.0*v_l))/(2.0*rover_hb));
  }

  //printf("%lf| %lf| %lf\n", rover_odo.linear.x, rover_odo.linear.y , rover_odo.angular.z);

  return;
}

/*******************************************
 * ROSメッセージ rover_sensor へのセンサ入力情報の入力
 */
void setRoverSensor(){

  rover_sensor.data[0] = wheel.u16Map(MU16_IM_DI);
  rover_sensor.data[1] = (uint16_t)(wheel.u16Map(MU16_M_VI)*1000/0xfff*56.1);

  return;
}
