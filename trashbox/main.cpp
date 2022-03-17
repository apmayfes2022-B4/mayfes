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


void set_up(Encoder,EKF){
    //セットアップ
    return;
}

void calc(Encoder &enc,EKF &ekf,vector<Vector3d> &states,Matrix3d &state_var_before,Vector2d &u,const Matrix<double,6,1> &y){//1ステップ計算する
    Vector3d state_before = states.back();
    ekf.input_estimate(states,u);
    // 観測更新
    auto K = ekf.est_update_gain(state_var_before);
    auto state_obs = ekf.est_update_state(state_before,u,y,K);
    auto state_var_middle = ekf.est_update_state_var(state_var_before,K);
    // 時間更新式
    auto state_after = ekf.state_evolution(state_obs,u);
    auto state_var_after = ekf.evo_update_state_var(state_var_middle,state_before,u);
    // x[4],Pともに更新
    states.erase(states.begin());
    states.push_back(state_after);
    state_var_before = state_var_after;
    return;
}

//グローバルに defineしたいんだけどどこに書くのがいいんだろうなあ

bool contenue = 1;



int main(int argc,char const *argv[]){
    //初期値
    double x_c = 0;
    double y_c = 0;
    double w_j = 0;


    // 定数の設定
    const double deltat=0.0; // EKFの更新間隔
    const Matrix<double,2,3> d;
    const Matrix<double,2,3> e;
    const double a=0;
    Matrix3d Xi;
    Matrix<double,6,6> Eta;




    //エンコーダ　と　カルマンフィルタのセットアップ
    vector<Vector3d> states(4); // 状態の初期設定(x,y,theta)
    Matrix3d state_var; // 共分散行列の初期設定
    Vector2d u;
    Matrix<double,6,1> y;
    Encoder enc(0,0,0);
    EKF ekf(deltat,d,e,a,Xi,Eta);
    set_up(enc,ekf);
    ekf.debug();
    //フィルタで計算

    while(contenue){
        cin >> x_c >> y_c >> w_j >> e_x >> e_y >> e_z >> cc;
        if(cc == 0){
            contenue = 0;
        }
        calc();


    }
    return 0;
}

//コンパイルはとりあえず g++ main.cpp -I includes/include.hh で