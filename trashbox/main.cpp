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

// あんまりここ(main.cpp)で定義してほしくないけど書くならここかな
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
    //初期値
    //書かなくていい気がしてきた。

    //エンコーダ　と　推定機　と　カルマンフィルタのセットアップ//多分もっと頭の良い書き方がある
    Encoder enc;
    Input_estimate est;
    EKF ekf;
    set_up(enc,est,ekf);
    
    //フィルタで計算
    
    while(contenue){
        double x_c,y_c,w_j;
        int e_x,e_y,e_z;
        bool cc;
        cin >> x_c >> y_c >> w_j >> e_x >> e_y >> e_z >> cc;
        if(cc == 0){
            contenue = 0;
        }
        
        enc.update(e_x,e_y,e_z);
        est.calc();
        //ekf.calc(enc,est);
    }
    
    return 0;
}

//コンパイルはとりあえず g++ main.cpp -I includes/include.hh -std=c++11 で