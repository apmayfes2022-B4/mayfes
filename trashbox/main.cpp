#include <iostream>
#include "Eigen/Dense"
#include <math.h>
#include <vector>
using namespace std;

#include "includes/include.hh"
/*
(目標)
入力が ./a.out 1 2 3 1 3 3 1 4 4 ...みたいなかんじに入ってきて
出力を　[1 1 1], [2 1 3], .. ってかえすことにしておく

現在どうすればいいかわかんない(on line 処理?)
*/


void set_up(Encoder enc,Input_estimate est,EKF ekf){
    ekf.u = est.u;
    //セットアップ ここで0以外の値をいれることにする。例としては ekf.y.x_c = 1;とか
    return;
}



//グローバルに defineしたい定数

bool go_on = 1;//入力がつづくかどうか

// あんまりここ(main.cpp)で定数を定義してほしくないけど書くならここかな


int main(int argc,char const *argv[]){
    //初期値//書かなくていい気がしてきた。

    //エンコーダ　と　推定機　と　カルマンフィルタのセットアップ     //多分もっと頭の良い書き方があるけど、とりまこれで
    Encoder enc;
    Input_estimate est;
    EKF ekf;
    set_up(enc,est,ekf);
    bool hoge;
    
    //逐一フィルタで計算
    while(go_on){
        // 観測
        double x_c,y_c,w_j;
        int e_x,e_y,e_z;
        bool cc;
        cin >> x_c >> y_c >> w_j >> e_x >> e_y >> e_z >> cc;//入力
        /*
        1.0 1.0 0.5 1 2 4 1
        2.0 0.0 0.5 2 2 3 1
        3.0 1.0 0.5 3 2 2 1
        4.0 1.0 0.5 4 2 1 0
        こんな感じの入力をしてデバッグしてみてください
        */

        if(cc == 0){
            go_on = 0;//終了判断
        }

        // 入力の推定
        enc.update(e_x,e_y,e_z);
        vector<double> omega(3);
        omega = enc.Omega();
        // ekfに反映
        ekf.y << x_c, y_c, ekf.y(2)+w_j, omega[0], omega[1], omega[2];
        ekf.output();
        auto temp = ekf.x_series;
        //推定(線形補間)の実行
        est.estimate(temp);
        est.debug();
        
        ekf.calc(enc,est);//ここの実装を期待してます。
        ekf.output();//出力

        /* 更新と補間がある？x時系列から求めるだけなので更新せず片方だけでいい気がしている
        Input next_est;
        next_est = ekf.next_est();
        est.update(next_est);//ekfの値から時系列を更新
        */
        //est.debug();
    }
    
    return 0;
}

//コンパイルはとりあえず g++ main.cpp -I includes/include.hh -std=c++11 で