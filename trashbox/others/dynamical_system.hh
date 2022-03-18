#ifndef DYNAMICAL_SYSTEM_CLASS__
#define DYNAMICAL_SYSTEM_CLASS__

#include "../Eigen/Dense"
using place = pair<double,double>;


class dynamical_system
{
public:
    double dt;

    void show_dt(){
        cout<<"dt"<<dt<<endl;
        return;
    }
};
#endif