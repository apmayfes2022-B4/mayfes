#ifndef ENCODER_CLASS__
#define ENCODER_CLASS__

#define initial 0 //初期値

#include "../Eigen/Dense"
#include "../others/dynamical_system.hh"
#include <iostream>


namespace Kalman
{
    typedef struct _scale{
        int x,y,z;
    }Scale;

    typedef struct _camera{
        double x,y;
    }Camera;

    class Encoder:dynamical_system
    {
    public:
    typedef struct{
        int x[2];
        int y[2];
        int z[2];
    }Time_series;

        Time_series time_series;

        Encoder(){
            time_series.x[0] = 0;
            time_series.x[1] = 0;
            time_series.y[0] = 0;
            time_series.y[1] = 0;
            time_series.z[0] = 0;
            time_series.z[1] = 0;
        }

        std::vector<double> Omega(){//角速度の計算
            std::vector<double> omega(3); 
            omega[0] = (time_series.x[1]-time_series.x[0])/dt;
            omega[1] = (time_series.y[1]-time_series.y[0])/dt;
            omega[2] = (time_series.z[1]-time_series.z[0])/dt;
            return omega;
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