/* 
    ロボットの中心が半径10の円を回るモデル
    出力は観測信号(steps,6)の行列
    コンパイルは g++ circle.cpp -I includes/include.hh -std=c++11 で
*/
/*
#include <iostream>
#include <vector>
#include "../Eigen/Dense"
#include <math.h>
#include <random>
using namespace std;

#include "../includes/include.hh"

#define PI 3.141592653589793
random_device seed;
mt19937 engine(seed());

using namespace Eigen;
using namespace Kalman;
//デバック用データ作成だから事故おこっても問題ないという認識

Matrix<double,6,1> random_noise(const double &sig=1e-1){
    normal_distribution<> dist(0.0, sig);
    Matrix<double,6,1> Eta = MatrixXd::Zero(6, 1);
    for (int i = 0; i < 6; i++){Eta(i) = dist(engine);}
    return Eta;
}

class Movement{
    public:
        typedef struct{
    double x;
    double y;
    double theta;
    }State_move;
        typedef struct{
    double v;
    double w;
    }Input_move;
    vector<State_move> state_series; 
    vector<Input_move> input_series; 
        int steps;
        Movement(const int &num_step){
            State_move init;
            init.x = 0; init.y = 0; init.theta = 0;
            state_series.push_back(init);
            steps = num_step;
        }
        void setting(const int &sig){
            for (int i = 0; i < steps-1; i++)
            {
                normal_distribution<> dist(0.0, sig);
                State_move state_i;
                State_move state_before = state_series.back();
                state_i.theta = 2.0*PI/steps*(i+1.0)+dist(engine);
                state_i.x = 10*cos(state_i.theta)+dist(engine);
                state_i.y = 10*sin(state_i.theta)+dist(engine);
                Input_move vel_i;
                double vel_vec[2];
                vel_vec[0] = state_i.x-state_before.x;
                vel_vec[1] = state_i.y-state_before.y;
                vel_i.v = sqrt(pow(vel_vec[0],2)+pow(vel_vec[1],2));
                vel_i.w = state_i.theta-state_before.theta;
                state_series.push_back(state_i);
                input_series.push_back(vel_i);
            }
        }
};

class X_Encode{
    public:
        double dt;
        double time_resolution;
        int x;
        int y;
        int z;
        X_Encode(){
            x=0;
            y=0;
            z=0;
        }
        void update(const vector<double> &omega, const Encoder &enc){
            if(omega.size()>=3){
                x+=(int)(omega[0]*dt/(2*PI/enc.time_resolution()));
                y+=(int)(omega[1]*dt/(2*PI/enc.time_resolution()));
                z+=(int)(omega[2]*dt/(2*PI/enc.time_resolution()));
            }
        }
};

int main(){
    EKF ekf;
    Encoder enc;
    ekf.u << 0,0;
    int steps = 1000;
    double sigma = 1e-2;    // 標準偏差
    Movement states(steps);
    states.setting(sigma);  //誤差あり時間発展
    int end = 1;
    X_Encode x_encode;

    char outputfilename[256];

    uint batch =1;
    sprintf(outputfilename, "../data/debug_data/batch_%u.dat", batch);
	std::ofstream outputfile (outputfilename, std::ios::out | std::ios::trunc);      //Output File (ofstream) 
	std::cout << "Writing details to file  : " << outputfilename << std::endl;				

    for (int i = 1; i < steps; i++)
    {
        ekf.x << states.state_series[i].x, states.state_series[i].y, states.state_series[i].theta;
        ekf.u << states.input_series[i].v, states.input_series[i].w;
        ekf.toy_evolution();
        ekf.y += random_noise(sigma);
        if(i==steps-1){end=0;}
        vector<double> omega(3);
        omega[0] = ekf.y(3); omega[1] = ekf.y(4); omega[2] = ekf.y(5);  
        x_encode.update(omega,enc);
        // cout << ekf.y.transpose() << endl; // y=(x,t,theta,omega0,omega1,omega2)の生データ
        outputfile << ekf.y(0) << " " << ekf.y(1) << " " << ekf.y(2) << " " 
        << x_encode.x << " " << x_encode.y << " " << x_encode.z << " " << end << endl; //誤差あり観測
    }
    
    return 0;
}
*/