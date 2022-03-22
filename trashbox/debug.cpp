#include "includes/include.hh"


void set_up(Kalman::Encoder enc,Kalman::Input_estimate est,Kalman::EKF ekf){
    //周期　このペースで入力が必ずあるものと仮定する
    //パラメタ==================================
    
    enc.set_dt(0.001);//高速入力
    est.set_dt(0.01);//数学的な計算過程
    ekf.set_dt(0.01);//数学的な計算過程
    
    //=========================================
    //セットアップ ここで0以外の値をいれることにする。例としては ekf.y.x_c = 1;とか
    return;
}

void calc(Kalman::Encoder enc,Kalman::Input_estimate est,Kalman::EKF ekf,Obs y_now,Kalman::Scale ee){
    est.update(ekf.get_x_series());
    ekf.update(enc,est,y_now);
    ekf.debug();
    //ekf.output();//x_k|kを出力
    return;
}


bool go_on = 1;

int main(){
    Kalman::Encoder enc;
    Kalman::Input_estimate est;
    Kalman::EKF ekf;
    set_up(enc,est,ekf);//初期値の設定　getcmdoptionの実装面倒なので許して
    //std::cout<< " "<<enc.get_dt() << " "<< std::endl;//est.get_dt()<<std::endl;
    
    Kalman::Camera came;
    Kalman::Scale ee;

    came.x = 0;came.y = 0;
    ee.x = 0;ee.y = 0; ee.z = 0;
    Obs y_now;
    
    //逐一フィルタで計算
    while(go_on==1){
        // 観測
        double input_type;//こいつで入力を管理
        std::cin >> input_type;//うまいことやってinput_typeでどっちが来たか判断
        if(input_type==0){//encorder
            std::cin >> ee.x >> ee.y >> ee.z;
            enc.update(ee);
            //enc.show_Omega();//エンコーダは大丈夫
        }else if(input_type == 1){//camera
            std::cin >> came.x >> came.y;
            y_now << came.x, came.y, 0, ee.x, ee.y, ee.z;
            calc(enc,est,ekf,y_now,ee);
        }else{
            go_on = 0;
        }
    }
    return 0;
}

//コンパイルはとりあえず g++ debug.cpp -I includes/include.hh -std=c++17 で