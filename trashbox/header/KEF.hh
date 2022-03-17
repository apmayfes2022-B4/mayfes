#ifndef EKF_CLASS__
#define EKF_CLASS__

#define State Vector3d
#define Obs Matrix<double,6,1>
#define Input Vector2d
#define State_Var Matrix3d
#define Gain Matrix<double,3,6>
#define PI 3.141592653589793

#define encoder int

#include <Eigen/Dense>

using std::cos;
using std::sin;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;


class EKF
{
    public:
        const double dt;   //時間間隔
        const Matrix<double,2,3> d;  //モータの位置ベクトル
        Matrix<double,2,3> e; //モータの進行方向ベクトル
        const double a; //回転半径
        const Matrix3d Xi;const Matrix<double,6,6> Eta;    //ノイズの共分散
        int x_,y_,w_j;
        encoder w;
        State x;
        Obs y;
        State_Var P_before_initial;

        EKF(const int &x_0, const int &y_0, const int &w_0, const double &Deltat){
            x_ = x_0;
            y_ = y_0;
            w_j = w_0;
            dt = Deltat;
        }

        void debug(){
            cout<<"Hello"<<endl;
        }
        
        State state_evolution(const State &x,const Input &u){
            State x_next;
            x_next(0) = x(0) + u(0) * cos(x(2))*dt;
            x_next(1) = x(1) + u(0) * sin(x(2))*dt;
            x_next(2) = x(2) + u(1) *dt;
            return x_next;
        }

        Obs observe_evolution(const State &x,const Input &u){
            Obs y;
            y(0) = x(0), y(1) = x(1), y(2)= u(1);
            for (int i = 0; i < 3; i++)
            {
                y[i+3] = enc_transformation(i,u);   
            }
            return y;
        }

        double enc_transformation(const int &i, const Input &u){
            auto pos = e(i,0)*u[0];
            auto e_i = e.col(i).transpose();
            auto vel = e_i*Rotation2d(PI/2.0)*d.col(i)*u[1];
            return (pos + vel)/a;
        }
        
        Matrix3d F_est(const State &x, const Input &u){
            Matrix3d F;
            F(0,0) = 1;
            F(0,2) = -u(0)*sin(x(2))*dt ;
            F(1,1) = 1;
            F(1,2) = u(0)*cos(x(2))*dt ;
            F(2,2) = 1;
            return F;
        }

        Matrix<double,6,3> H_est(void){
            Matrix<double,6,3> H;
            H(0,0) = 1;
            H(1,1) = 1;
            return H;
        }

        Gain est_update_gain(const State_Var &P_before){
            return P_before*H_est().transpose()*(H_est()*P_before*H_est().transpose()+Eta);
        }

        State est_update_state(const State &x, const Input &u, const Obs &y,const Gain &K){
            return x + K * (y-observe_evolution(x,u));
        }

        State_Var est_update_state_var(const State_Var &state_before, const Gain &K){
            return state_before - K*H_est().transpose()*state_before;
        }

        State_Var evo_update_state_var(const State_Var &state_after, const Matrix3d &F){
            return F*state_after*F.transpose()+Xi;
        }
        
};

Matrix2d Rotation2d(const double theta){
            Matrix2d R;
            R << cos(theta),-sin(theta),
                sin(theta),cos(theta);
            return R;
};
#endif

