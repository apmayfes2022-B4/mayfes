#include <MsTimer2.h>

volatile int i = 0;  //02
int outX = 5;  // X軸のピン番号
int outY = 4;  // Y軸のピン番号
float input_volt = 5.0; // Arduinoの電源電圧
long zeroOmegaX = 0;
long zeroOmegaY = 0;

// 同一ディレクトリ内で同じ名前の関数を2つつくるとエラーが発生する？
// というか，同一ディレクトリ内のファイルは同時にマイコンボードに書き込まれる？
void read_gyro() {
  //一定時間毎にここが呼び出される
  long valX = analogRead(outX);
  long valY = analogRead(outY);

  // Arduinoのビットのまま出力
  valX = valX - zeroOmegaX;
  valY = valY - zeroOmegaY;
  Serial.print("X=");
  Serial.print(valX);
  Serial.print(",Y=");
  Serial.println(valY);

  // 電圧にしたのち角速度deg/secに変換．不要ならコメントアウト.
  float f_valX = valX;
  float f_valY = valY;
  float voltX = input_volt * f_valX / 1024;
  float angvelX = voltX * 1000 / 6.7;
  float voltY = input_volt * f_valY / 1024;
  float angvelY = voltY * 1000 / 6.7;
  Serial.print("angvelX=");
  Serial.print(angvelX);
  Serial.print(",angvelY=");
  Serial.println(angvelY);
  
}

void training() {
  // ノイズが入って動かしていなくても値があるので
  // 基準点を学習する？
  delay(1000);
  for (i=0; i<500; i++){
    zeroOmegaX = zeroOmegaX + analogRead(outX);
    zeroOmegaY = zeroOmegaY + analogRead(outY);
  }
  zeroOmegaX = zeroOmegaX / 500;
  zeroOmegaY = zeroOmegaY / 500;
}

void setup() {

  Serial.begin(38400);

  //analogpinってOUTPUTとかINPUTの設定いらないのか

  training();

  //1000ms毎にtimer発火
  MsTimer2::set(1000, read_gyro);
  MsTimer2::start();
}

void loop() {
  
}
