#ifndef DYNAMICAL_SYSTEM_CLASS__
#define DYNAMICAL_SYSTEM_CLASS__

#include "../Eigen/Dense"

namespace Kalman
{
    class dynamical_system
    {
    private:
        double dt;
        double time_resolution;
    public:
        void set_dt(double time){
            dt = time;
            return;
        }

        void set_time_resolution(double time){
            time_resolution = time;
            return;
        }

       
        double get_dt(){
            return dt;
        }

        double get_time_resolution(){ return time_resolution; }

    };
}
#endif