#include <iostream>
#include "Eigen/Dense"
#include <math.h>

using namespace std;

#include "includes/include.hh"
/*
(目標)
入力が ./a.out 1 2 3 1 3 3 1 4 4 ...みたいなかんじに入ってきて
出力を　[1 1 1], [2 1 3], .. ってかえすことにしておく

現在どうすればいいかわかんない(on line 処理?)
*/



void set_up(Encoder,EKF){
    //セットアップ
    return;
}

void calc(){//1ステップ計算する
    return;
}

//グローバルに defineしたいんだけどどこに書くのがいいんだろうなあ



int main(int argc,char const *argv[]){
    //初期値
    double x_c = 0;
    double y_c = 0;
    double w_j = 0;
    int e_x = 0;
    int e_y = 0;
    int e_z = 0;
    //エンコーダ　と　カルマンフィルタのセットアップ
    Encoder enc(0,0,0);
    EKF ekf(0,0,0,1e-3);
    set_up(enc,ekf);


    ekf.debug();
    //フィルタで計算
    while(1==0){
        calc();
    }
    return 0;
}

//コンパイルはとりあえず g++ main.cpp -I includes/include.hh で