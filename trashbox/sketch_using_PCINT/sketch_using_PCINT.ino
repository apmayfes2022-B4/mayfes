
#include <TimerOne.h> //高性能なタイマーライブラリの読み込み
#include <Encoder.h>
#include "PinChangeInterrupt.h"

//3つよみとりたい

Encoder myEnc1(4, 5);//A相,B相の組み合わせ(2,3以外、なんでもいい)
Encoder myEnc2(6, 7);//A相,B相の組み合わせ(2,3以外、なんでもいい)
Encoder myEnc3(8, 9);//A相,B相の組み合わせ(2,3以外、なんでもいい)
volatile long oldPosition1  = -999;
volatile long newPosition1  = 0;
volatile long oldPosition2  = -999;
volatile long newPosition2  = 0;
volatile long oldPosition3  = -999;
volatile long newPosition3  = 0;

void RotEnc1() {
  newPosition1 = myEnc1.read();
  if (newPosition1 != oldPosition1) {
    oldPosition1 = newPosition1;
    Serial.println(newPosition1);
  }
}

void RotEnc2() {
  newPosition2 = myEnc2.read();
  if (newPosition2 != oldPosition2) {
    oldPosition2 = newPosition2;
    Serial.println(newPosition2);
  }
}

void RotEnc3() {
  newPosition3 = myEnc3.read();
  if (newPosition3 != oldPosition3) {
    oldPosition3 = newPosition3;
    Serial.println(newPosition3);
  }
}

void setup() {
  Serial.begin(9600);
  attachPCINT(digitalPinToPCINT(4),RotEnc1,CHANGE);
  attachPCINT(digitalPinToPCINT(6),RotEnc2,CHANGE);
  attachPCINT(digitalPinToPCINT(8),RotEnc3,CHANGE);
}


void loop() {
}
