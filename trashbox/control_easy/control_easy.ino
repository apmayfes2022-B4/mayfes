
// 向きの変化がめっちゃちっちゃいと思うことにする(sin(delta_theta) \approx delta_theta)
// この近似ができないなら車体の半径がエンコーダの出力いくつ分に相当するか知る必要あり．

#define PI 3.141592653589793

#include <Encoder.h>
#include "PinChangeInterrupt.h"

// モーターのピン番号
int MOTOR_1A = 3;
int MOTOR_1B = 5;
int MOTOR_2A = 6;
int MOTOR_2B = 9;
int MOTOR_3A = 10;
int MOTOR_3B = 11;

// エンコーダのピン番号
int ENCODER_1A = 0;
int ENCODER_1B = 4;
int ENCODER_2A = 7;
int ENCODER_2B = 8;
int ENCODER_3A = 12;
int ENCODER_3B = 13;

// エンコーダの設定
Encoder myEnc1(ENCODER_1A, ENCODER_1B);//A相,B相の組み合わせ(2,3以外、なんでもいい)
Encoder myEnc2(ENCODER_2A, ENCODER_2B);//A相,B相の組み合わせ(2,3以外、なんでもいい)
Encoder myEnc3(ENCODER_3A, ENCODER_3B);//A相,B相の組み合わせ(2,3以外、なんでもいい)

// エンコーダの位置を表す変数
volatile long oldPosition1  = -999;
volatile long newPosition1  = 0;
volatile long oldPosition2  = -999;
volatile long newPosition2  = 0;
volatile long oldPosition3  = -999;
volatile long newPosition3  = 0;

// 向きのPI制御で使う変数
volatile float Rtheta = 0;
volatile float delta_Rtheta_sum = 0;

// 向きのPI制御で使う定数
const float Kp_direction = 1;
const float Ki_direction = 1;

// モータの出力
volatile long motor_val1 = 0;
volatile long motor_val2 = 0;
volatile long motor_val3 = 0;

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
  analogWrite(MOTOR_1A, max(motor1, 0));
  analogWrite(MOTOR_1B, max(-motor1, 0));
  analogWrite(MOTOR_2A, max(motor2, 0));
  analogWrite(MOTOR_2B, max(-motor2, 0));
  analogWrite(MOTOR_3A, max(motor3, 0));
  analogWrite(MOTOR_3B, max(-motor3, 0));
}

void velocity_bangbang(start_x, start_y, target_x, target_y) {
  // ここに速度に関する制御の初期化を書く．
  // 3つのモーターに対する出力の総和は0になる．
  // やればすぐできる，はず．
  float delta_x = target_x - start_x;
  float delta_y = target_y - start_y;
  motor_val1 = delta_y;
  motor_val2 = (sqrt(3)*delta_x - delta_y)/2;
  motor_val3 = (- sqrt(3)*delta_x - delta_y)/2;

  max_motor_val = max(motor_val1, motor_val2, motor_val3);
  motor_val1 = 255 * motor_val1 / max_motor_val;
  motor_val2 = 255 * motor_val2 / max_motor_val;
  motor_val3 = 255 * motor_val3 / max_motor_val;

  motor_output(motor_val1, motor_val2, motor_val3);
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

  motor_output(motor_val1, motor_val2, motor_val3);
}

void setup() {
  // pinmodeの設定
  // エンコーダ
  pinMode(ENCODER_1A, INPUT);
  pinMode(ENCODER_1B, INPUT);
  pinMode(ENCODER_2A, INPUT);
  pinMode(ENCODER_2B, INPUT);
  pinMode(ENCODER_3A, INPUT);
  pinMode(ENCODER_3B, INPUT);

  // モーター
  pinMode(MOTOR_1A, OUTPUT);
  pinMode(MOTOR_1B, OUTPUT);
  pinMode(MOTOR_2A, OUTPUT);
  pinMode(MOTOR_2B, OUTPUT);
  pinMode(MOTOR_3A, OUTPUT);
  pinMode(MOTOR_3B, OUTPUT);
  
  velocity_bangbang(start_x, start_y, target_x, target_y);
}


void loop() {
  PI_control_direction();
}
