#ifndef ENCODER_CLASS__
#define ENCODER_CLASS__

//#include "../Eigen/Dense"
#include "../others/dynamical_system.hh"
//#include <iostream>


namespace Kalman
{
    typedef struct{
        int x,y,z;
    }Scale;

<<<<<<< HEAD
    typedef struct{
=======
    typedef struct _camera{
>>>>>>> 367bb4c44e37e17fa568278f03d66efea0749324
        double x,y;
    }Camera;

    class Encoder:public dynamical_system
    {
    private:
    typedef struct{
        int x[2];
        int y[2];
        int z[2];
    }Time_series;

        Time_series time_series;
        double trans_coefficient;//エンコーダ目盛りを回転角度に翻訳

    public:
        Encoder(){
            time_series.x[0] = 0;
            time_series.x[1] = 0;
            time_series.y[0] = 0;
            time_series.y[1] = 0;
            time_series.z[0] = 0;
            time_series.z[1] = 0;
            set_dt(0.001);
            trans_coefficient = 0.01;
        }

        std::vector<double> Omega(){//角速度の計算
            std::vector<double> omega(3); 
            double delta_t = get_dt();
            omega[0] = (time_series.x[1]-time_series.x[0])/delta_t*trans_coefficient;
            omega[1] = (time_series.y[1]-time_series.y[0])/delta_t*trans_coefficient;
            omega[2] = (time_series.z[1]-time_series.z[0])/delta_t*trans_coefficient;
            return omega;
        }

        void show_Omega(){
            double delta_t = get_dt();
            std::cout<<"omega[0] = "<<(time_series.x[1]-time_series.x[0])/delta_t*trans_coefficient<<"omega[1] = "<<(time_series.y[1]-time_series.y[0])/delta_t*trans_coefficient<<"omega[2] = "<<(time_series.z[1]-time_series.z[0])/delta_t*trans_coefficient<<std::endl;
            return;
        }

        void update(Scale ee){
            // time_seriesの更新
            time_series.x[0] = time_series.x[1];
            time_series.y[0] = time_series.y[1];
            time_series.z[0] = time_series.z[1];
            time_series.x[1] = ee.x;
            time_series.y[1] = ee.y;
            time_series.z[1] = ee.z;
            return;
        }

        void debug(){
            std::cout<<"previously, x: "<<time_series.x[0]<<", y: "<<time_series.y[0]<<", z: "<<time_series.z[0]<<std::endl;
            std::cout<<"x: "<<time_series.x[1]<<", y: "<<time_series.y[1]<<", z: "<<time_series.z[1]<<std::endl;
            return ;
        }
    };
}
#endif