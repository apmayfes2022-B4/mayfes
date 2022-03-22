#include "includes/include.hh"
/*
(目標)
入力が ./a.out 1 2 3 1 3 3 1 4 4 ...みたいなかんじに入ってきて
出力を　[1 1 1], [2 1 3], .. ってかえすことにしておく
*/


void set_up(Kalman::Encoder enc,Kalman::Input_estimate est,Kalman::EKF ekf){
    //周期　このペースで入力が必ずあるものと仮定する
    //パラメタ==================================
    enc.dt = 0.001;//高速入力
    est.dt = 0.01;//数学的な計算過程
    ekf.dt = 0.01;//数学的な計算過程,カメラの周波数に対応
    //=========================================
    //セットアップ ここで0以外の値をいれることにする。例としては ekf.y.x_c = 1;とか
    return;
}

void calc(Kalman::Encoder enc,Kalman::Input_estimate est,Kalman::EKF ekf,Obs y_now,Kalman::Scale ee){
    //enc.update(ee);
    est.update(ekf.x_series);
    ekf.update(enc,est,y_now);//ここの実装を期待してます。//推定(線形補間)の実行もここの中
    ekf.output();//x_k|kを出力
    return;
}


//グローバルに defineしたい定数
bool go_on = true;//入力がつづくかどうか
int main(int argc,char const *argv[]){
    //エンコーダ　と　推定機　と　カルマンフィルタのセットアップ     //多分もっと頭の良い書き方があるけど、とりまこれで
    Kalman::Encoder enc;
    Kalman::Input_estimate est;
    Kalman::EKF ekf;
    set_up(enc,est,ekf);//初期値の設定　getcmdoptionの実装面倒なので許して

    Kalman::Camera came;
    Kalman::Scale ee;
    Obs y_now;
    came.x = 0;came.y = 0;
    ee.x = 0;ee.y = 0; ee.z = 0;
    double input_type;//こいつで入力を管理
    input_type = 0;
    
    //逐一フィルタで計算
    while(go_on==1){
        // 観測
<<<<<<< HEAD
        std::cin >> input_type;//うまいことやってinput_typeでどっちが来たか判断
        if(input_type==0){//encorder
            std::cin >> ee.x >> ee.y >> ee.z;
            y_now << came.x, came.y, 0, ee.x, ee.y, ee.z;//とりあえず　ジャイロは0
            enc.update(ee);
            enc.debug();
        }else if(input_type == 1){//camera
            std::cin >> came.x >> came.y;
            y_now << came.x, came.y, 0, ee.x, ee.y, ee.z;
            calc(enc,est,ekf,y_now,ee);
        }else{
            go_on = 0;
=======
        Kalman::Camera came;
        double w_j;
        Kalman::Scale ee;
        bool cc;
        std::cin >> came.x >> came.y >> w_j >> ee.x >> ee.y >> ee.z >> cc;//入力
        Obs y_now;
        y_now << came.x, came.y, ekf.y(2)+w_j, ee.x, ee.y, ee.z;//ekf.y(2)+w_j
        /*
        1.0 1.0 0.5 1 2 4 1
        2.0 0.0 0.5 2 2 3 1
        3.0 1.0 0.5 3 2 2 1
        4.0 1.0 0.5 4 2 1 0
        10.0258 0.0581536 0.015383 1 0 0 1
        こんな感じの入力をしてデバッグしてみてください
        */

        if(cc == 0){
            go_on = false;//終了判断
>>>>>>> 367bb4c44e37e17fa568278f03d66efea0749324
        }
    }
    return 0;
}
//コンパイルはとりあえず g++ main.cpp -I includes/include.hh -std=c++17 で
