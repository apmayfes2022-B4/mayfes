#ifndef ENCODER_CLASS__
#define ENCODER_CLASS__

#define initial 0 //初期値

#include "../others/dynamical_system.hh"
#include <iostream>


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

    void update(int e_x,int e_y,int e_z){
        // time_seriesの更新
        time_series.x[0] = time_series.x[1];
        time_series.y[0] = time_series.y[1];
        time_series.z[0] = time_series.z[1];
        time_series.x[1] = e_x;
        time_series.y[1] = e_y;
        time_series.z[1] = e_z;
        return;
    }

    void debug(){
        //cout<<"x"<<time_series.x[0]<<" y"<<time_series.y[0]<<" z"<<time_series.z[0]<<endl;
        cout<<"x"<<time_series.x[1]<<" y"<<time_series.y[1]<<" z"<<time_series.z[1]<<endl;
        return ;
    }
};
#endif