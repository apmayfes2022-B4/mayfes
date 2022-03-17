#ifndef INPUT_CLASS__
#define INPUT_CLASS__

#define vel double
#define ang double
#include <Eigen/Dense>


class INPUT_EST
{
public:
    int x[2],y[2],z[2];

    INPUT_EST(int x_0 = 0,int y_0 = 0,int z_0 = 0){
        x[1] = x_0;
        y[1] = y_0;
        z[1] = z_0;
    }

    

};

#endif