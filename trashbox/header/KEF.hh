#ifndef EKF_CLASS__
#define EKF_CLASS__

#define PI 3.141592653589793


#include "../Eigen/Dense"
#include "../others/dynamical_system.hh"
#include <iostream>

/*
#define Obs Matrix<double,6,1>
#define Input Vector2d
#define State_Var Matrix3d
#define Gain Matrix<double,3,6>
#define State Vector3d
using std::cos;
using std::sin;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
*/
//この上で宣言されている型何もわからないので、適当に置き換えています。あとで修正お願いします。

typedef struct _Observable{
    double x_c;//カメラ x
    double y_c;//カメラ y
    double wj;//ジャイロセンサ
    vector<double> th{0,0,0};
}Observable;
typedef struct _State{
    double x;
    double y;
    double z;
}State;


class EKF:dynamical_system
{

private:
    //車のモデルはここ
    class Car
    {
    public:
        //Matrix<double,2,3> d;  //モータの位置ベクトル
        //Matrix<double,2,3> e; //モータの進行方向ベクトル
        vector<place> d = std::vector<place>(3);
        vector<place> e = std::vector<place>(3);
        
        double a; //回転半径
        Car(){//初期化
            a = 1;
            place d1 = make_pair(0,0);
            place d2 = make_pair(0,0);
            place d3 = make_pair(0,0);
            d[1] = d1;
            d[2] = d2;
            d[3] = d3;
            //Matrix<double,2,3>
        }

        void update(){//こいつで位置ベクトルと進行方向ベクトルを更新
            return;
        }
    };
public:

    Observable y;
    State x;
    Input u;
        
    //Matrix3d Xi;Matrix<double,6,6> Eta;    //ノイズの共分散
    //State_Var P_before_initial;

    //初期化　construct
    EKF(){//とりあえず全部0で初期化しておきます。0以外を入れたい時はmain.cpp の　set_up()で入れることにしましょう。
        y.x_c = 0;
        y.y_c = 0;
        y.th = {0,0,0};
        x.x = 0;
        x.y = 0;
        x.z = 0;
        /*
        dt = deltat;
        d = d_const;
        e = e_const;
        a = a_const;
        Xi = Xi_const;
        Eta = Eta_const;
        */
    }

    void debug(){
        cout<<"Hello"<<endl;
        return;
    }

    void calc(Encoder encoder,Input_estimate est){//encoder.x, encoder.yの更新を行う
        u = est.u;//EKFの入力にInput_estimateの予測値を代入した

        //ここに下でコメントアウトした部分をうまいこといれてください。
    /*
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
            return (pos + vel(0))/a;
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
            return P_before*H_est().transpose()*(H_est()*P_before*H_est().transpose()).inverse()+Eta;
        }

        State est_update_state(const State &x, const Input &u, const Obs &y,const Gain &K){
            return x + K * (y-observe_evolution(x,u));
        }

        State_Var est_update_state_var(const State_Var &state_before, const Gain &K){
            return state_before - K*H_est().transpose()*state_before;
        }

        State_Var evo_update_state_var(const State_Var &state_after, const State &x,const Input &u){
            auto F = F_est(x,u);
            return F*state_after*F.transpose()+Xi;
        }

        void input_estimate(const vector<Vector3d> &states,Vector2d &u){
            // Euler method
            Vector3d state_4 = states[3];// latest
            Vector3d state_3 = states[2];// semi-latest
            auto vel = state_4-state_3;
            u[0] = sqrt(pow(vel[0],2)+pow(vel[1],2));
            u[1] = state_4(2)-state_3(2);
        }

        void calc(Encoder &enc,EKF &ekf,vector<Vector3d> &states,Matrix3d &state_var_before,Vector2d &u,const Matrix<double,6,1> &y){//1ステップ計算する
            Vector3d state_before = states.back();
            ekf.input_estimate(states,u);
            // 観測更新
            auto K = ekf.est_update_gain(state_var_before);
            auto state_obs = ekf.est_update_state(state_before,u,y,K);
            auto state_var_middle = ekf.est_update_state_var(state_var_before,K);
            // 時間更新式
            auto state_after = ekf.state_evolution(state_obs,u);
            auto state_var_after = ekf.evo_update_state_var(state_var_middle,state_before,u);
            // x[4],Pともに更新
            states.erase(states.begin());
            states.push_back(state_after);
            state_var_before = state_var_after;
            return;
        }
    */
        //
        return;
    }

    Input next_est(){
        Input next_u;
        next_u.v = u.v;
        next_u.w = u.w;
        return next_u;
    }

    void output(){//状態の出力
        cout<<x.x<<" "<<x.y<<" "<<x.z<<endl;
        return;
    }

    
};
/*
Matrix2d Rotation2d(const double theta){
            Matrix2d R;
            R << cos(theta),-sin(theta),
                sin(theta),cos(theta);
            return R;
};
*/
#endif

