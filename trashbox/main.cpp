#include "includes/include.hh"
/*
(目標)
入力が ./a.out 1 2 3 1 3 3 1 4 4 ...みたいなかんじに入ってきて
出力を　[1 1 1], [2 1 3], .. ってかえすことにしておく

(on line 処理?)
*/


void set_up(Encoder enc,Input_estimate est,EKF ekf){
    //ekf.u = est.u;
    //セットアップ ここで0以外の値をいれることにする。例としては ekf.y.x_c = 1;とか
    return;
}

void calc(Encoder enc,Input_estimate est,EKF ekf,Obs y_now,Scale ee){
    enc.update(ee);
    est.update(ekf.x_series);
    ekf.update(enc,est,y_now);//ここの実装を期待してます。//推定(線形補間)の実行もここの中
    ekf.output();//x_k|kを出力
    return;
}

//グローバルに defineしたい定数
bool go_on = true;//入力がつづくかどうか
int main(int argc,char const *argv[]){

    //エンコーダ　と　推定機　と　カルマンフィルタのセットアップ     //多分もっと頭の良い書き方があるけど、とりまこれで
    Encoder enc;
    Input_estimate est;
    EKF ekf;
    set_up(enc,est,ekf);
    //bool hoge;//これなんですか？
    
    //逐一フィルタで計算
    while(go_on==1){
        // 観測
        Camera came;
        double w_j;
        Scale ee;
        bool cc;
        cin >> came.x >> came.y >> w_j >> ee.x >> ee.y >> ee.z >> cc;//入力

        Obs y_now;
        y_now << came.x, came.y, ekf.y(2)+w_j, ee.x, ee.y, ee.z;//ekf.y(2)+w_j

        /*
        1.0 1.0 0.5 1 2 4 1
        2.0 0.0 0.5 2 2 3 1
        3.0 1.0 0.5 3 2 2 1
        4.0 1.0 0.5 4 2 1 0
        こんな感じの入力をしてデバッグしてみてください
        */

        if(cc == 0){
            go_on = false;//終了判断
        }
        calc(enc,est,ekf,y_now,ee);
    }
    
    return 0;
}

//コンパイルはとりあえず g++ main.cpp -I includes/include.hh -std=c++17 で

//あと知ってるかもしれないですけど/**/で複数行コメントアウトができますよ