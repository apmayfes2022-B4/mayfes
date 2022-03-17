#ifndef INPUT_ESTIMATE_CLASS__
#define INPUT_ESTIMATE_CLASS__

#include "../others/dynamical_system.hh"
#include <iostream>

typedef struct{
    double v;//速度の絶対値
    double w;//角速度
}Input;

// w,vの推定(線形補間)を行う
class Input_estimate:dynamical_system
{
public:
typedef struct{
    double v[2];//速度の絶対値
    double w[2];//角速度
}Time_series_double;

    Input u;//予測値
    Time_series_double time_series_double;//時系列

    Input_estimate(){//初期化
        time_series_double.v[0] = 0;
        time_series_double.w[0] = 0;
        time_series_double.v[1] = 0;
        time_series_double.w[1] = 0;
    }

    
    void update(Input next_est){//時系列を更新
        time_series_double.v[0] = time_series_double.v[1];
        time_series_double.w[0] = time_series_double.w[1];
        time_series_double.v[1] = next_est.v;
        time_series_double.w[1] = next_est.w;
        return;
    }

    
    void estimate(){//予測値を更新
        u.v = time_series_double.v[1]*2-time_series_double.v[0];//time_series_double.v[1]+(time_series_double.v[1]-time_series_double.v[0])/dt*dt;
        u.w = time_series_double.w[1]*2-time_series_double.w[0];//time_series_double.w[1]+(time_series_double.w[1]-time_series_double.w[0])/dt*dt;
        return;
    }
    

    void debug(){
        cout<<"v"<<time_series_double.v[0]<<" "<<time_series_double.v[1]<<endl;
    }
};
#endif