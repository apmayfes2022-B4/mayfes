#ifndef INPUT_ESTIMATE_CLASS__
#define INPUT_ESTIMATE_CLASS__

#include "../others/dynamical_system.hh"
#include <iostream>

typedef struct{
    double v;//速度の絶対値
    double w;//角速度
}Input;

// w,vの推定(線形補間)を行う
class Input_estimate:dynamical_system
{
public:
    Input u[2];

    Input_estimate(){//初期化
        u[0].v = 0;
        u[0].w = 0;
        u[1].v = 0;
        u[1].w = 0;
    }

    void calc(){//u,wを更新
        /*
        double v_next;
        double w_next; 
        v_next = u[1].v+(u[1].v-u[0].v)/dt*dt;
        w_next = u[1].w+(u[1].w-u[0].w)/dt*dt;
        double v_temp,w_temp;
        v_temp = u[1].v;
        w_temp = u[1].w;
        u[0].v = v_temp;
        u[0].w = w_temp;
        u[1].v = v_next;
        u[1].w = w_next;
        */
        return ;
    }

    void debug(){
        cout<<"Input_estimate"<<endl;
    }
};
#endif