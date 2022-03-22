
// 向きの変化がめっちゃちっちゃいと思うことにする(sin(delta_theta) \approx delta_theta)
// この近似ができないなら車体の半径がエンコーダの出力いくつ分に相当するか知る必要あり．

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

// 向きのPI制御で使う変数
float Rtheta = 0;
float delta_Rtheta_sum = 0;
// 向きのPI制御で使う定数
float Kp_direction = 1;
float Ki_direction = 1;

long motor_val1 = 0;
long motor_val2 = 0;
long motor_val3 = 0;

void update_encoder() {
  oldPosition1 = newPosition1;
  oldPosition2 = newPosition2;
  oldPosition3 = newPosition3;
  newPosition1 = myEnc1.read();
  newPosition2 = myEnc2.read();
  newPosition3 = myEnc3.read();
}

void motor_output(motor1, motor2, motor3) {
  // 単純にモーターに値を入力する関数．
  // ちょっとモーター出力をどういうふうにするか理解できてないので現状空欄．
  // どれくらいの値を入力すればどれだけ回るのかてきな．
}

void velocity_bangbang(start_x, start_y, target_x, target_y) {
  // ここに速度に関する制御の初期化を書く．
  // 3つのモーターに対する出力の総和は0になる．
  // やればすぐできる，はず．
}

void PI_control_direction() {
  float enc_diff1 = newPosition1 - oldPosition1;
  float enc_diff2 = newPosition2 - oldPosition2;
  float enc_diff3 = newPosition3 - oldPosition3;

  float delta_Rtheta = (enc_diff1 + enc_diff2 + enc_diff3) / 3;
  Rtheta = Rtheta + delta_Rtheta;
  delta_Rtheta_sum = delta_Rtheta_sum + (0 - Rtheta);

  motor_val1 = motor_val1 + Kp_direction * (0 - Rtheta) + Ki_direction * delta_Rtheta_sum;
  motor_val2 = motor_val2 + Kp_direction * (0 - Rtheta) + Ki_direction * delta_Rtheta_sum;
  motor_val3 = motor_val3 + Kp_direction * (0 - Rtheta) + Ki_direction * delta_Rtheta_sum;
}

void setup() {
  velocity_bangbang(start_x, start_y, target_x, target_y);
}


void loop() {
  PI_control_direction();
}
