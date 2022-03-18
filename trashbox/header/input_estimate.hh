#ifndef INPUT_ESTIMATE_CLASS__
#define INPUT_ESTIMATE_CLASS__

#include "../others/dynamical_system.hh"
#include <iostream>
#include <vector>
#include "../Eigen/Dense"

using std::cout;
using std::endl;
using std::vector;
using Eigen::Vector2d;
using Eigen::Vector3d;

// typedef struct{
//     double v;//速度の絶対値
//     double w;//角速度
// }Input;
#define Input Vector2d
#define State Vector3d


// w,vの推定(線形補間)を行う
class Input_estimate:dynamical_system
{
public:
// typedef struct{
//     double v[2];//速度の絶対値
//     double w[2];//角速度
// }Time_series_double;

    Input u;//予測値
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //おまじない
    // Time_series_double time_series_double;//時系列

    // Input_estimate(){//初期化
    //     time_series_double.v[0] = 0;
    //     time_series_double.w[0] = 0;
    //     time_series_double.v[1] = 0;
    //     time_series_double.w[1] = 0;
    // }

    Input_estimate(){//0で初期化
        u << 0, 0;
    }
    
    // void update(Input next_est){//時系列を更新
    //     time_series_double.v[0] = time_series_double.v[1];
    //     time_series_double.w[0] = time_series_double.w[1];
    //     time_series_double.v[1] = next_est.v;
    //     time_series_double.w[1] = next_est.w;
    //     return;
    // }
    
    // void estimate(){//予測値を更新
    //     u.v = time_series_double.v[1]*2-time_series_double.v[0];//time_series_double.v[1]+(time_series_double.v[1]-time_series_double.v[0])/dt*dt;
    //     u.w = time_series_double.w[1]*2-time_series_double.w[0];//time_series_double.w[1]+(time_series_double.w[1]-time_series_double.w[0])/dt*dt;
    //     return;
    // }
    
    void estimate(const vector<State,Eigen::aligned_allocator<State> > &state_series){// 型がバグるので取り敢えず（要修正）
        auto latest = state_series.back();
        auto sz = state_series.size();
        auto semilatest = state_series[sz-2];
        auto vel = latest-semilatest;
        u[0] = sqrt(pow(vel[0],2)+pow(vel[1],2));// v
        u[1] = vel(2);// w
    }

    void debug(){
        cout << "debugging input estimation" << endl;
        cout << u.transpose() << endl;
        // cout<<"v"<<time_series_double.v[0]<<" "<<time_series_double.v[1]<<endl;
    }
};
#endif