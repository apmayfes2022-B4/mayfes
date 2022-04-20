
#define PI 3.141592653589793

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
volatile float theta = 0;

void update_encoder() {
  oldPosition1 = newPosition1;
  oldPosition2 = newPosition2;
  oldPosition3 = newPosition3;
  newPosition1 = myEnc1.read();
  newPosition2 = myEnc2.read();
  newPosition3 = myEnc3.read();
}

void predict_position() {
  volatile long delta_enc_1 = newPosition1 - oldPosition1;
  volatile long delta_enc_2 = newPosition2 - oldPosition2;
  volatile long delta_enc_3 = newPosition3 - oldPosition3;
  
  float diff_enc_12 = delta_enc_1 - delta_enc_2;
  float diff_enc_13 = delta_enc_1 - delta_enc_3;
  
  float a = - sin(theta) + sin(theta+2*PI/3);
  float b = cos(theta) - cos(theta+2*PI/3);
  float c = - sin(theta) + sin(theta-2*PI/3);
  float d = cos(theta) - cos(theta-2*PI/3);
  float delta_x = (d * diff_enc_12 - b * diff_enc_13) / (a*d - b*c);
  float delta_y = (- c * diff_enc_12 + a * diff_enc_13) / (a*d - b*c);
}

void setup() {
  Serial.begin(9600);
  attachPCINT(digitalPinToPCINT(4),RotEnc1,CHANGE);
  attachPCINT(digitalPinToPCINT(6),RotEnc2,CHANGE);
  attachPCINT(digitalPinToPCINT(8),RotEnc3,CHANGE);
}


void loop() {
  update_encoder();
  predict_position();
}
