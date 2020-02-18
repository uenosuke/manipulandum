// GR-PEACHボードを使ったmanipulandum制御用プログラムです
// SPI通信で絶対値エンコーダのデータを取得し角度を初期化，その後はインクリメンタルエンコーダで角度を取得します
// モータドライバはESCONで，PWMにて速度指令を出します．10%～90%で-300~+300rpm の範囲で制御できる設定です
// 作成日 2020年02月14日
// 作成者 uenosuke

#include <Arduino.h>
#include <MsTimer2.h>
#include <SPI.h>

#include "define.h"
#include "phaseCounterPeach.h"
#include "AMT203VPeach.h"
#include "SDclass.h"
#include "LCDclass.h"
#include "Button.h"
#include "Filter.h"

#define RPM2PWM ( 128.0/300.0 )

phaseCounter enc1(1);
phaseCounter enc2(2);

AMT203V absenc1(&SPI, PIN_CSB1);
AMT203V absenc2(&SPI, PIN_CSB2);
mySDclass mySD;
myLCDclass myLCD(&SERIAL_LCD);

Filter filterFx(INT_TIME);
Filter filterFy(INT_TIME);

// グローバル変数の設定
coords tipPosi = {0.0, 0.0, 0.0};
coords tipVel = {0.0, 0.0, 0.0};
coords refVel, pre_refVel = {0.0, 0.0, 0.0};
coords PA_mass = {20.0, 20.0, 0.0}; // パワーアシストの慣性項
coords PA_viscos = {30.0, 30.0, 0.0}; // パワーアシストの粘性項
coords rawF, fltrd_rawF, fltrd_localF, gF;
double theta1, theta2;//関節角度
double pre_theta1, pre_theta2;
double theta1_0, theta2_0;// 初期関節角度
double act_omega1, act_omega2;
double ref_omega1, ref_omega2;
double ref_rpm1, ref_rpm2;
int ref_pwm1, ref_pwm2;
int encount1 = 0, encount2 = 0;
int fxad_neutral, fyad_neutral; // 力センサのAD変換値の中立点

bool flag_10ms = false; // loop関数で10msごとにシリアルプリントできるようにするフラグ
bool flag_100ms = false;

// LEDをチカチカさせるための関数
void LEDblink(byte pin, int times, int interval){
  analogWrite(pin, 0);
  for(int i = 0; i < times; i++){
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}

// setupで有効にされるタイマ割り込み処理が書いてある場所
void timer_warikomi(){    
  // RGB LED を良い感じに光らせるための処理
  static int count = 0;
  static int count_flag = 0;
  count += 2; // ここで光る周期を変えられる(はず)
  count_flag++;

  if(count < 255){
    analogWrite(PIN_LED_RED, count);
    analogWrite(PIN_LED_BLUE, 255 - count);
  }else if(count < 255 * 2){
    analogWrite(PIN_LED_GREEN, count - 255);
    analogWrite(PIN_LED_RED, 255 * 2 - count);
  }else if(count < 255 * 3){
    analogWrite(PIN_LED_BLUE, count - 255 * 2);
    analogWrite(PIN_LED_GREEN, 255 * 3 - count);
  }else{
    count = 0;
  }

  // フラグ立てるための処理
  flag_10ms = true;
  if(count_flag >= 10){
    flag_100ms = true;
    count_flag = 0;
  }

  double C1, C2, C12, S1, S2, S12;
  //int encount1, encount2; // X,Y軸エンコーダのカウント値
  // 自己位置推定用エンコーダのカウント値取得
  encount1 = enc1.getCount();
  encount2 = enc2.getCount();

  theta1 = -(theta1_0 + encount1) / (4096.0 * PI); // encountをもとに現在の角度を計算(回転方向に合わせてマイナス付けた)
  theta2 = -(theta2_0 + encount2) / (4096.0 * 2.0 * PI); // 肘関節はセンサ直結(回転方向に合わせてマイナス付けた)

  C1 = cos(theta1);
  C2 = cos(theta2);
  C12 = cos(theta1 + theta2);
  S1 = sin(theta1);
  S2 = sin(theta2);
  S12 = sin(theta1 + theta2);

  tipPosi.x =  L_ARM * (C1 + C12);
  tipPosi.y =  L_ARM * (S1 + S12);

  act_omega1 = (theta1 - pre_theta1) / INT_TIME; // 各関節の角速度を計算
  act_omega2 = (theta2 - pre_theta2) / INT_TIME;

  tipVel.x = - L_ARM * ((S1 + S12)*act_omega1 + S12 * act_omega2);
  tipVel.x =   L_ARM * ((C1 + C12)*act_omega1 + C12 * act_omega2);

  rawF.x = (analogRead(PIN_FX) - fxad_neutral) * 0.02442; //200 / 2047.5; // 力は0~3.3Vを12ビットに変換されて取得できる
  rawF.y = (analogRead(PIN_FY) - fyad_neutral) * 0.02442; // 力は±200N
  fltrd_rawF.x = filterFx.LowPassFilter(rawF.x); // センサの生データをフィルターにかける（ノイズ除去）
  fltrd_rawF.y = filterFy.LowPassFilter(rawF.y);
  //fltrd_localF.x = (-fltrd_rawF.x + fltrd_rawF.x) / 1.41421356; // 力をアームの座標系に合わせて変換する
  //fltrd_localF.y = (-fltrd_rawF.x - fltrd_rawF.x) / 1.41421356;
  gF.x = fltrd_rawF.x * C1 - fltrd_rawF.y * S1; // 力を，マニピュランダムのグローバル座標系に変換
  gF.y = fltrd_rawF.x * S1 + fltrd_rawF.y * C1;

  coords PA_K = {1.0/PA_viscos.x, 1.0/PA_viscos.y, 0.0};
  coords PA_T = {PA_mass.x/PA_viscos.x, PA_mass.y/PA_viscos.y, 0.0};

  refVel.x = (PA_K.x * INT_TIME * gF.x + PA_T.x * pre_refVel.x)/(PA_T.x + INT_TIME);
  refVel.y = (PA_K.y * INT_TIME * gF.y + PA_T.y * pre_refVel.y)/(PA_T.y + INT_TIME);

  ref_omega1 =  (C12 * refVel.x + S12 * refVel.y)/(L_ARM * S2);
  ref_omega2 = -(-(C1 + C12) * refVel.x + (S1 + S12) * refVel.y)/(L_ARM * S2); //回転方向に合わせてマイナス付けた

  ref_rpm1 = ref_omega1 * 30 / PI; // 60/2PI だけど，30/PIで
  ref_rpm2 = ref_omega2 * 30 / PI;

  ref_pwm1 = ref_rpm1 * RPM2PWM + 128;
  ref_pwm2 = ref_rpm2 * RPM2PWM + 128;

  if(ref_pwm1 < 0) ref_pwm1 = 0;
  if(ref_pwm1 > 255) ref_pwm1 = 255;
  if(ref_pwm2 < 0) ref_pwm2 = 0;
  if(ref_pwm2 > 255) ref_pwm2 = 255;

  analogWrite(PIN_ESCON1_PWM, ref_pwm1);
  analogWrite(PIN_ESCON2_PWM, ref_pwm2);

  pre_refVel = refVel;
  pre_theta1 = theta1;
  pre_theta2 = theta2;

  //int absencount1 = absenc1.getEncount();
  //int absencount2 = absenc2.getEncount(); 
  //theta1 = (double)absencount1 / (4096) * PI;	// 角度に変換
  //theta2 = (double)absencount2 / (4096) * PI;	// 角度に変換
  
}

// エラーが発生したら無限ループで停止
void error_stop(){
  myLCD.clear_display();
  myLCD.write_line("     !ERROR!", LINE_1);
  while(1){
    analogWrite(PIN_LED_RED, 255);
    analogWrite(PIN_LED_BLUE, 0);
    wait(0.25);
    analogWrite(PIN_LED_RED, 0);
    analogWrite(PIN_LED_BLUE, 255);
    wait(0.25);
  }
}

void setup()
{
  bool ready_to_start = false;
  bool setting_sequence = false;
  //int lcd_line_num = LINE_0;
  //String lcd_message = "";

  Serial.begin(115200);
  /*SERIAL_LCD.begin(115200);
  SERIAL_XBEE.begin(115200);
  pinMode(PIN_XBEERESET, OUTPUT); // XBeeのリセット
  digitalWrite(PIN_XBEERESET, 0);
  delay(10);
  digitalWrite(PIN_XBEERESET,1);
  delay(10);*/
  
  pinMode(PIN_SW, INPUT); // オンボードのスイッチ
  pinMode(PIN_LED_USER, OUTPUT); // オンボードのLED赤

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  pinMode(PIN_LED_ENC, OUTPUT);
  
  pinMode(31, INPUT);
  pinMode(PIN_SW_RIGHT, INPUT);
  pinMode(PIN_SW_LEFT, INPUT);
  pinMode(PIN_SW_DOWN, INPUT);
  pinMode(PIN_SW_A, INPUT);
  pinMode(PIN_SW_B, INPUT);
  
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  analogWrite(PIN_LED_RED, 0); // 消しちゃダメ，ぜったい →　LPMSのために
  analogWrite(PIN_LED_BLUE, 0);
  analogWrite(PIN_LED_GREEN, 0);
  analogWrite(PIN_ESCON1_PWM, 128);
  analogWrite(PIN_ESCON2_PWM, 128);

  analogReference(RAW12BIT);
  
  //myLCD.color_white(); // LCDの色を白に
  //myLCD.clear_display(); // LCDをクリア

  SPI.begin(); // ここでSPIをbeginしてあげないとちゃんと動かなかった
  SPI.setClockDivider(SPI_CLOCK_DIV16); //SPI通信のクロックを1MHzに設定 beginの後に置かないと，処理が止まる
  Serial.print(absenc1.init());
  Serial.print(" ");
  Serial.println(absenc2.init());

  // AMT203V関係
  enc1.init();
  enc2.init();
  int absencount1 = absenc1.getEncount();
  int absencount2 = absenc2.getEncount(); 
  theta1 = pre_theta1 = theta1_0 = (double)absencount1 / (4096) * PI;	// 角度に変換
  theta2 = pre_theta2 = theta2_0 = (double)absencount2 / (4096) * PI;	// 角度に変換

  Serial.println("initial count");
  Serial.print(absencount1);
  Serial.print(", ");
  Serial.println(absencount2);

  Serial.println("initial angle");
  Serial.print(theta1);
  Serial.print(", ");
  Serial.println(theta2);
  LEDblink(PIN_LED_GREEN, 2, 100);

  //mySD.init();
  //delay(10);
  //myLCD.write_line("SD-card initialized", LINE_3);
  //mySD.make_logfile();
  //LEDblink(PIN_LED_RED, 2, 100);

  // ボード上の"A"ボタンが押されるまで待機
  while(digitalRead(PIN_SW_A)){
    delay(10);
  }

  fxad_neutral = analogRead(PIN_FX);
  filterFx.setLowPassPara(0.02, 0.0); // ローパスフィルタ(ノイズ除去用)を初期化
  fyad_neutral = analogRead(PIN_FY);
  filterFy.setLowPassPara(0.02, 0.0);
  LEDblink(PIN_LED_RED, 2, 100);

  /*myLCD.clear_display();
  myLCD.write_line("# Program Started  #", LINE_1);
  myLCD.write_line("pX:      pY:", LINE_2);
  
  delay(750); 

  myLCD.write_line("Angle:", LINE_3);

  myLCD.write_double(gPosi.x, LINE_2, 3);
  myLCD.write_double(gPosi.y, LINE_2, 12);
  myLCD.write_double(gPosi.z, LINE_3, 6);*/

  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();
}

void loop()
{
  // オンボードエンコーダやスイッチの状態に応じてLEDを制御
  if(digitalRead(31) == 1){
    digitalWrite(PIN_LED_1, 1);
  }else{
    digitalWrite(PIN_LED_1, 0);
  }
  if(digitalRead(PIN_SW_LEFT) == 1){
    digitalWrite(PIN_LED_2, 1);
  }else{
    digitalWrite(PIN_LED_2, 0);
  }
  if(digitalRead(PIN_SW_RIGHT) == 1){
    digitalWrite(PIN_LED_3, 1);
  }else{
    digitalWrite(PIN_LED_3, 0);
  }
  if(digitalRead(PIN_SW_DOWN) == 1){
    digitalWrite(PIN_LED_4, 1);
    //digitalWrite(PIN_LED_ENC, 1);
  }else{
    digitalWrite(PIN_LED_4, 0);
    //digitalWrite(PIN_LED_ENC, 0);
  }

  // 10msに1回ピン情報を出力する
  if(flag_10ms){
    //coords refV = controller.getRefVel(LJoyX, LJoyY, RJoyY); // ジョイスティックの値から，目標速度を生成
    //platform.VelocityControl(refV); // 目標速度に応じて，プラットフォームを制御

    // SDカードにログを吐く
    /*String dataString = "";
    static bool first_write = true;
    if(first_write){
      dataString += "gPosix,gPosiy,gPosiz,refVx,refVy,refVz";
      mySD.write_logdata(dataString);
      first_write = false;
      dataString = "";
    }
    dataString += String(gPosi.x, 4) + "," + String(gPosi.y, 4) + "," + String(gPosi.z, 4);
    dataString += "," + String(refV.x, 4) + "," + String(refV.y, 4) + "," + String(refV.z, 4);

    mySD.write_logdata(dataString);*/

    Serial.print(theta1_0);
    Serial.print("\t");
    Serial.print(theta2_0);
    Serial.print("\t");
    Serial.print(rawF.x);
    Serial.print("\t");
    Serial.print(rawF.y);
    Serial.print("\t");
    Serial.print(fltrd_localF.x);
    Serial.print("\t");
    Serial.println(fltrd_localF.y);

    
    /*Serial.print(analogRead(A0));
    Serial.print(" ");
    Serial.print(analogRead(A1));
    Serial.print(" ");
    Serial.println(analogRead(A4));*/

    flag_10ms = false;
  }

  // 100msごとにLCDを更新する
  if(flag_100ms){
    //myLCD.write_double(gPosi.x, LINE_2, 3);
    //myLCD.write_double(gPosi.y, LINE_2, 12);
    //myLCD.write_double(gPosi.z, LINE_3, 6);
    
    flag_100ms = false;
  }
 //delayMicroseconds(100);
}

