#ifndef DEFINE_h
#define DEFINE_h

#include "Arduino.h"

struct coords{
    double x;
    double y;
    double z;
};

#define L_ARM ( 0.400 ) // アーム長

#define SERIAL_LCD      Serial6
#define SERIAL_XBEE     Serial7

#define PIN_XBEERESET ( 66 )

// スイッチやLEDのピン設定
#define PIN_DIP1 ( 25 )
#define PIN_DIP2 ( 24 )
#define PIN_DIP3 ( 69 )
#define PIN_DIP4 ( 70 )

#define PIN_SW_UP    ( 31 )
#define PIN_SW_LEFT  ( 6 )
#define PIN_SW_RIGHT ( 7 )
#define PIN_SW_DOWN  ( 30 )

#define PIN_SW_A  ( 29 )
#define PIN_SW_B  ( 28 )

#define PIN_ENC_A  ( 26 )
#define PIN_ENC_B  ( 27 )

#define PIN_LED_1 ( 20 )
#define PIN_LED_2 ( 36 )
#define PIN_LED_3 ( 37 )
#define PIN_LED_4 ( 38 )
#define PIN_LED_ENC ( 40 )

#define PIN_CSB1 ( 33 )
#define PIN_CSB2 ( 32 )

#define PIN_ESCON1_PWM ( 4 )
#define PIN_ESCON2_PWM ( 5 )

#define PIN_FX ( 14 )//A0
#define PIN_FY ( 15 )//A1
#define PIN_MZ ( 18 )//A4

// 制御周期
#define INT_TIME			( 0.01 )//( 0.001 )

#endif
