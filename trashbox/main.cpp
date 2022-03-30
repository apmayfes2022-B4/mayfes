#include "includes/include.hh"
/*
(目標)
入力が ./a.out 1 2 3 1 3 3 1 4 4 ...みたいなかんじに入ってきて
出力を　[1 1 1], [2 1 3], .. ってかえすことにしておく
*/


void set_up(Kalman::Encoder enc,Kalman::EKF ekf){
    //周期　このペースで入力が必ずあるものと仮定する
    //パラメタ==================================
    //=========================================
    //セットアップ ここで0以外の値をいれることにする。例としては ekf.y.x_c = 1;とか
    return;
}



//グローバルに defineしたい定数
bool go_on = true;//入力がつづくかどうか
bool awake = 0;//カメラデータが入ったかどうか

int main(int argc,char const *argv[]){
    //エンコーダ　と　推定機　と　カルマンフィルタのセットアップ     //多分もっと頭の良い書き方があるけど、とりまこれで
    Kalman::Encoder enc;
    Kalman::EKF ekf;
    //set_up(enc,ekf);//初期値の設定　getcmdoptionの実装面倒なので許して

    Kalman::Camera came;
    Kalman::Scale ee;
    Obs y_now;
    came.x = 0;came.y = 0;
    ee.x = 0;ee.y = 0; ee.z = 0;
    std::vector<double> omega(3,0);
    double input_type;//こいつで入力を管理
    input_type = 0;
    double t_now = 0;//Timerの予定　getTimeNowみたいなものを実装
    double t_before = 0;
    double t_diff = 0;
    
    //逐一フィルタで計算
    while(go_on==1){
        // 観測
        std::cin >> input_type;//うまいことやってinput_typeでどっちが来たか判断
        t_before = t_now;
        t_now += 0.001;//getTimeNow()
        t_diff = t_now - t_before;
        if(input_type == 0){//encorder
            std::cin >> ee.x >> ee.y >> ee.z;
            enc.set_t_diff(t_diff);
            enc.update(ee);
            omega = enc.Omega();
            y_now << came.x, came.y, omega[0], omega[1], omega[2];
            ekf.set_t_diff(t_diff);
            ekf.update(input_type,y_now);
        }else if(input_type == 1){//camera
            std::cin >> came.x >> came.y;
            if(awake==0){
                awake = 1;
                ekf.set_up_ekf(came.x,came.y);
            }
            y_now << came.x, came.y,omega[0], omega[1], omega[2];
            ekf.set_t_diff(t_diff);
            ekf.update(input_type,y_now);
        }else{
            go_on = 0;
        }
        ekf.output();
        //ekf.debug();
    }
    return 0;
}
//コンパイルはとりあえず g++ main.cpp -I includes/include.hh -std=c++17 で
