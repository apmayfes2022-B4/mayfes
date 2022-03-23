
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

// 速さのP制御で使う定数
volatile float Kp_velocity = 1;

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

void PI_control_direction(int & direction_correct) {
  float enc_diff1 = newPosition1 - oldPosition1;
  float enc_diff2 = newPosition2 - oldPosition2;
  float enc_diff3 = newPosition3 - oldPosition3;

  float delta_Rtheta = (enc_diff1 + enc_diff2 + enc_diff3) / 3;
  Rtheta = Rtheta + delta_Rtheta;
  delta_Rtheta_sum = delta_Rtheta_sum + (0 - Rtheta);

  direction_correct = Kp_direction * (0 - Rtheta) + Ki_direction * delta_Rtheta_sum;
  // 255より大きかったり-255より小さかったりすると bangbang_and_P_control_velocity でバグる．
  direction_correct = min(direction_correct, 255);
  direction_correct = max(direction_correct, -255);
}

void bangbang_and_P_control_velocity(direction_correct) {
  // ここに速度に関する制御の初期化を書く．
  // 3つのモーターに対する出力の総和は0になる．
  float delta_x = target_x - start_x;
  float delta_y = target_y - start_y;
  float distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

  // P制御
  motor_val1 = Kp_velocity * delta_y;
  motor_val2 = Kp_velocity * (sqrt(3)*delta_x - delta_y) / 2;
  motor_val3 = Kp_velocity * (- sqrt(3)*delta_x - delta_y) / 2;

  // オーバーフローする場合はオーバーフローするギリギリにする．結果的にbangbang制御になる．
  // ここで決めた出力にdirection_correctが加算されることに注意する．
  // ところで abs(direction_correct)が255よりデカいとバグる気がする．
  long available_min = - 255 - direction_correct;
  long available_max = 255 - direction_correct;
  float min_motor_val = min(motor_val1, motor_val2, motor_val3);
  float max_motor_val = max(motor_val1, motor_val2, motor_val3);
  if (min_motor_val < available_min or max_motor_val > available_max) {
    float bangbang_ratio = min(available_max/max_motor_val, available_min/min_motor_val);
    motor_val1 = bangbang_ratio * motor_val1;
    motor_val2 = bangbang_ratio * motor_val2;
    motor_val3 = bangbang_ratio * motor_val3;
  }
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
}


void loop() {
  // cur_x, cur_y （自己位置推定班）から取得する関数をここに入れる．
  // start_x, start_y, target_x, target_y （ソフトウェア班）を毎ループ更新する関数をここに入れる．そのままの値でも良い．あるいは変更時に割り込みでもよい．

  update_encoder();
  
  // 向きの補正のために各モーターの出力に加算する値
  float direction_correct = 0;
  PI_control_direction(direction_correct);

  // direction_correctの値を用いてオーバーフローしないように次に出力する速さを決定
  bangbang_and_P_control_velocity(direction_correct);
  motor_val1 = motor_val1 + direction_correct; 
  motor_val2 = motor_val2 + direction_correct; 
  motor_val3 = motor_val3 + direction_correct; 
  
  motor_output(motor_val1, motor_val2, motor_val3);
}
