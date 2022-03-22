#ifndef INPUT_ESTIMATE_CLASS__
#define INPUT_ESTIMATE_CLASS__

//#include "../Eigen/Dense"
#include "../others/dynamical_system.hh"

namespace Kalman
{
    // w,vの推定(線形補間)を行う
    class Input_estimate:public dynamical_system
    {
    private:
    typedef struct{
        double v[2];//速度の絶対値
        double w[2];//角速度
    }Time_series_double;

        Input u;//予測値
        Time_series_double time_series_double;//時系列

    public:

        Input_estimate(){//初期化
            time_series_double.v[0] = 0;
            time_series_double.w[0] = 0;
            time_series_double.v[1] = 0;
            time_series_double.w[1] = 0;
            set_dt(0.01);
            u << 0, 0;
        }

        Input get_input(){
            return u;
        }
    
        void update(const std::vector<State> &x_series){//時系列を更新
            time_series_double.v[0] = time_series_double.v[1];
            time_series_double.w[0] = time_series_double.w[1];
            time_series_double.v[1] = u(0);
            time_series_double.w[1] = u(1);
            int sz = x_series.size();
            u(0) = (x_series[sz-1](0)-x_series[sz-2](0))/get_dt();//time_series_double.v[1]+(time_series_double.v[1]-time_series_double.v[0])/dt*dt;
            u(1) = (x_series[sz-1](1)-x_series[sz-2](1))/get_dt();//time_series_double.w[1]+(time_series_double.w[1]-time_series_double.w[0])/dt*dt;
            return;
        }
    
        void debug(){
            std::cout << "debugging input estimation" << std::endl;
            std::cout << u.transpose() << std::endl;
        }
    };
}
#endif