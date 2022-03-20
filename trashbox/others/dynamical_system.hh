#ifndef DYNAMICAL_SYSTEM_CLASS__
#define DYNAMICAL_SYSTEM_CLASS__

#include "../Eigen/Dense"
//using place = pair<double,double>;

namespace Kalman
{
    class dynamical_system
    {
    public:
        double dt;

        void show_dt(){
            std::cout<<"dt"<<dt<<std::endl;
            return;
        }
    };
}
#endif