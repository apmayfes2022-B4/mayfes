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
    //セットアップ ここで0以外の値をいれることにする。例としては ekf.y.x_c = 1;とか
    return;
}



//グローバルに defineしたい定数

bool contenue = 1;//入力がつづくかどうか

// あんまりここ(main.cpp)で定数を定義してほしくないけど書くならここかな
/*
    const double deltat=0.0; // EKFの更新間隔
    const Matrix<double,2,3> d;
    const Matrix<double,2,3> e;
    const double a=0;
    Matrix3d Xi;
    Matrix<double,6,6> Eta;

    vector<Vector3d> states(4); // 状態の初期設定(x,y,theta)
    Matrix3d state_var; // 共分散行列の初期設定
    Vector2d u;
    Matrix<double,6,1> y;
*/



int main(int argc,char const *argv[]){
    //初期値//書かなくていい気がしてきた。

    //エンコーダ　と　推定機　と　カルマンフィルタのセットアップ     //多分もっと頭の良い書き方があるけど、とりまこれで
    Encoder enc;
    Input_estimate est;
    EKF ekf;
    set_up(enc,est,ekf);
    
    //逐一フィルタで計算
    while(contenue){
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
            contenue = 0;//終了判断
        }

        enc.update(e_x,e_y,e_z);
        est.estimate();//推定(線形補間)の実行
        //ekf.calc(enc,est);//ここの実装を期待してます。
        ekf.output();//出力

        Input next_est;
        next_est = ekf.next_est();
        est.update(next_est);//ekfの値から時系列を更新
        //est.debug();
    }
    
    return 0;
}

//コンパイルはとりあえず g++ main.cpp -I includes/include.hh -std=c++11 で