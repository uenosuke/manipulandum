// GR-PEACHボードを使ったプログラムです
// このプログラムは，Leonardo micro pro に接続されたコントローラを使った，全方向移動ロボットの制御プログラムです．
// 駆動方式は，オムニホイール3輪による全方向移動です．
// RoboClawは本ボードに取り付けられており，RoboClawライブラリの使用を前提とします
// http://downloads.basicmicro.com/code/arduino.zip
// 作成日 2019年12月30日
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

phaseCounter enc1(1);
phaseCounter enc2(2);

AMT203V absenc1(&SPI, PIN_CSB1);
AMT203V absenc2(&SPI, PIN_CSB2);
mySDclass mySD;
myLCDclass myLCD(&SERIAL_LCD);

/*Button button_up(PIN_SW_UP);
Button button_down(PIN_SW_DOWN);
Button button_left(PIN_SW_LEFT);
Button button_right(PIN_SW_RIGHT);
Button button_B(PIN_SW_A);
Button button_A(PIN_SW_B);
Button dip1(PIN_DIP1);
Button dip2(PIN_DIP2);
Button dip3(PIN_DIP3);
Button dip4(PIN_DIP4);*/

// グローバル変数の設定
coords gPosi = {0.0, 0.0, 0.0};
double theta1, theta2;
int encount1 = 0, encount2 = 0;

bool flag_10ms = false; // loop関数で10msごとにシリアルプリントできるようにするフラグ
bool flag_100ms = false;

// 最大最小範囲に収まるようにする関数
double min_max(double value, double minmax)
{
  if(value > minmax) value = minmax;
  else if(value < -minmax) value = -minmax;
  return value;
}

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

  double angle_rad;
  //int encount1, encount2; // X,Y軸エンコーダのカウント値
  // 自己位置推定用エンコーダのカウント値取得
  encount1 = enc1.getCount();
  encount2 = enc2.getCount();

  int absencount1 = absenc1.getEncount();
  int absencount2 = absenc2.getEncount(); 
  theta1 = (double)absencount1 / (4096) * PI;	// 角度に変換
  theta2 = (double)absencount2 / (4096) * PI;	// 角度に変換
  
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
  theta1 = (double)absencount1 / (4096) * PI;	// 角度に変換
  theta2 = (double)absencount2 / (4096) * PI;	// 角度に変換

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

  // コントローラの"A"ボタンが押されるまで待機
  while(digitalRead(PIN_SW_A)){
    delay(10);
  }

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

    Serial.print(theta1);
    Serial.print(" ");
    Serial.print(theta2);
    Serial.print(" ");
    Serial.print(absenc1.getEncount());
    Serial.print(" ");
    Serial.println(absenc2.getEncount());
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

