#ifndef INPUT_ESTIMATE_CLASS__
#define INPUT_ESTIMATE_CLASS__

#include "../Eigen/Dense"
#include "../others/dynamical_system.hh"

namespace Kalman
{
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
            dt = 0.01;
            u << 0, 0;
        }

    
        void update(const std::vector<State> &x_series){//時系列を更新
            time_series_double.v[0] = time_series_double.v[1];
            time_series_double.w[0] = time_series_double.w[1];
            time_series_double.v[1] = u(0);
            time_series_double.w[1] = u(1);
            int sz = x_series.size();//x_seriesが空の場合まずいので後で対応
            u(0) = (x_series[sz-1](0)-x_series[sz-2](0))/dt;//time_series_double.v[1]+(time_series_double.v[1]-time_series_double.v[0])/dt*dt;
            u(1) = (x_series[sz-1](1)-x_series[sz-2](1))/dt;//time_series_double.w[1]+(time_series_double.w[1]-time_series_double.w[0])/dt*dt;
            return;
        }
    
        void debug(){
            std::cout << "debugging input estimation" << std::endl;
            std::cout << u.transpose() << std::endl;
        // cout<<"v"<<time_series_double.v[0]<<" "<<time_series_double.v[1]<<endl;
        }
    };
}
#endif