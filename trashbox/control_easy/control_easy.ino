
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

// 位置に関する変数
volatile float start_x = 0;
volatile float start_y = 0;
volatile float target_x = 0;
volatile float target_y = 0;
volatile float cur_x = 0;
volatile float cur_y = 0;

// 速さのP制御で使う変数（P制御の定数だが，初速によってきめる）
volatile float Kp_motor1vel = 0;
volatile float Kp_motor2vel = 0;
volatile float Kp_motor3vel = 0;

// 位置に関する定数
const float POSITION_EPSILON = 1.0;// 目的地までどれくらい近くなったら止まるか

// すでにとまったかどうか
volatile int stopped = 0;

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
  float delta_x = target_x - start_x;
  float delta_y = target_y - start_y;
  float distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
  motor_val1 = delta_y;
  motor_val2 = (sqrt(3)*delta_x - delta_y)/2;
  motor_val3 = (- sqrt(3)*delta_x - delta_y)/2;

  max_motor_val = max(motor_val1, motor_val2, motor_val3);
  motor_val1 = 255 * motor_val1 / max_motor_val;
  motor_val2 = 255 * motor_val2 / max_motor_val;
  motor_val3 = 255 * motor_val3 / max_motor_val;

  // 初速を計算したのでP制御の定数が決定できる
  Kp_motor1vel = motor_val1 / distance;
  Kp_motor2vel = motor_val2 / distance;
  Kp_motor3vel = motor_val3 / distance;

  motor_output(motor_val1, motor_val2, motor_val3);
}

void stop_or_not(cur_x, cur_y, target_x, target_y) {
  // 目的位置に十分近くなったら止まる
  if (pow(cur_x-target_x,2) + pow(cur_y-target_y,2) < POSITION_EPSILON){
    stopped = 1;
    motor_output(0,0,0);
  }
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

void P_control_velocity() {
  motor_val1 = motor_val1 
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

  // start_x, start_y, target_x, target_y を取得する（ソフトウェア班から)
  
  velocity_bangbang(start_x, start_y, target_x, target_y);
}


void loop() {
  if (stopped == 0){

    // cur_x, cur_yを取得する（自己位置推定班から）

    P_control_velocity();// 速さに関するP制御．最悪これだけコメントアウトしても動くはず．
    PI_control_direction();// 向きに関するPI制御（速さに関するP制御で得た値を回転するように上書き）
    stop_or_not(cur_x, cur_y, target_x, target_y);// ゴール近くについたら止める
    motor_output(motor_val1, motor_val2, motor_val3);
  }
}
