#ifndef EKF_CLASS__
#define EKF_CLASS__

#define PI 3.141592653589793


#include "../Eigen/Dense"
#include "../others/dynamical_system.hh"
#include <iostream>

#define Obs Matrix<double,6,1>
#define Input Vector2d
#define State_Var Matrix3d
#define Gain Matrix<double,3,6>
#define State Vector3d
#define MotorPos vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >
#define MotorMove vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >

using std::cos;
using std::sin;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
//この上で宣言されている型何もわからないので、適当に置き換えています。あとで修正お願いします。

// typedef struct _Observable{
//     double x_c;//カメラ x
//     double y_c;//カメラ y
//     double wj;//ジャイロセンサ
//     vector<double> th{0,0,0};
// }Observable;
// typedef struct _State{
//     double x;
//     double y;
//     double z;
// }State;



class EKF:dynamical_system
{

private:
    //車のモデルはここ
    class Car
    {
    public:
        MotorPos d;  //モータの位置ベクトル
        MotorMove e; //モータの進行方向ベクトル
        // vector<place> d = std::vector<place>(3);
        // vector<place> e = std::vector<place>(3);
        Matrix3d Xi; Matrix<double,6,6> Eta;    //ノイズの共分散  
        double a; //回転半径
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Car(){//初期化
            a = 1;
            Vector2d d1(1.0,0);
            Vector2d d2(-1.0,0);
            Vector2d d3(0,1.71);
            d.push_back(d1) ; d.push_back(d2); d.push_back(d3); //静的確保がよくわからんのでしゃあなしで
            Vector2d e1(0.86,0.5);  //(sqrt{3}/2, 1/2)
            Vector2d e2(-0.86,-0.5);
            Vector2d e3(0.5,-0.86);
            e.push_back(e1) ; e.push_back(e2); e.push_back(e3); 
            MatrixXd xi = MatrixXd::Identity(3,3);
            Xi = xi;
            MatrixXd eta = MatrixXd::Identity(6,6);
            Eta = eta;
        }

        void update(){//こいつで位置ベクトルと進行方向ベクトルを更新
        /*
            ここ何を書くんですか
        */
            return;
        }
    };
public:

    Obs y;
    State x;
    Input u;
    vector<State,Eigen::aligned_allocator<State> > x_series; // 状態の時系列 
    vector<State_Var,Eigen::aligned_allocator<State_Var> > P_series; // 状態の時系列 
    State_Var P;
    Car car;
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //初期化　construct
    EKF(){//とりあえず全部0で初期化しておきます。0以外を入れたい時はmain.cpp の　set_up()で入れることにしましょう。
        y << 0, 0, 0, 0, 0, 0;
        x << 0, 0, 0;
        P << 0, 0, 0,
             0, 0, 0,
             0, 0, 0;
        VectorXd zero_vec = VectorXd::Zero(3);
        MatrixXd zero_mat = MatrixXd::Zero(3,3);
        x_series.push_back(x);
        P_series.push_back(P);
        for (int i = 0; i < 3; i++){x_series.push_back(zero_vec);P_series.push_back(zero_mat);}// 時系列を空で入れておく（3は{好きな長さ-1}）
    }

    void debug(){
        cout << "debuggin EKF" << endl;
        cout << "y is " << y.transpose() << "x is " << x.transpose() 
        << "u is " << u.transpose() << "P is " << P << endl;
        for(auto hoge:x_series){
            cout << "x-series are " << hoge.transpose() << endl;
        }
        for(auto fuga:P_series){
            cout << "P-series are " << fuga << endl;
        }
        return;
    }

    //ここに下でコメントアウトした部分をうまいこといれてください。
    State state_evolution(const State &x, const Input &u){
        State x_next;
        x_next(0) = x(0) + u(0) * cos(x(2))*dt;
        x_next(1) = x(1) + u(0) * sin(x(2))*dt;
        x_next(2) = x(2) + u(1) *dt;
        return x_next;
    }

    Obs observe_evolution(const State &x, const Input &u){
        Obs y;
        y(0) = x(0), y(1) = x(1), y(2)= u(1);
        for (int i = 0; i < 3; i++)
        {
            y[i+3] = enc_transformation(i,u);   
        }
        return y;
    }

    double enc_transformation(const int &i, const Input &u){
        auto pos = car.e[i](0)*u(0);
        auto e_i = car.e[i].transpose();
        auto vel = e_i*Rotation2d(PI/2.0)*car.d[i]*u(0);
        return (pos + vel(0))/car.a;
    }
    
    Matrix3d F_est(const State &x, const Input &u){
        Matrix3d F = MatrixXd::Zero(3, 3);
        F(0,0) = 1;
        F(0,2) = -u(0)*sin(x(2))*dt ;
        F(1,1) = 1;
        F(1,2) = u(0)*cos(x(2))*dt ;
        F(2,2) = 1;
        return F;
    }

    Matrix<double,6,3> H_est(void){
        Matrix<double,6,3> H = MatrixXd::Zero(6, 3);
        H(0,0) = 1;
        H(1,1) = 1;
        return H;
    }

    Gain est_update_gain(const State_Var &P_before){
        auto content_of_Inv = H_est()*P_before*H_est().transpose()+car.Eta;
        return P_before*H_est().transpose()*(content_of_Inv).inverse();
    }

    State est_update_state(const State &x, const Input &u, const Obs &y,const Gain &K){
        return x + K * (y-observe_evolution(x,u));
    }

    State_Var est_update_state_var(const State_Var &P_before, const Gain &K){
        return P_before - K*H_est()*P_before;
    }

    State_Var evo_update_state_var(const State_Var &x_after, const State &x,const Input &u){
        auto F = F_est(x,u);
        return F*x_after*F.transpose()+car.Xi;
    }

    void calc(Encoder enc, Input_estimate est){//1ステップ計算する
        u = est.u;
        Vector3d P_before = x_series.back();
        // ekf.input_estimate(states,u);
        // 観測更新
        auto K = est_update_gain(P);
        auto x_middle = est_update_state(x,u,y,K);
        auto P_middle = est_update_state_var(P,K);
        // 時間更新式
        auto x_after = state_evolution(x_middle,u);
        auto P_after = evo_update_state_var(P_middle,P_before,u);
        // x[4],Pともに更新
        x_series.erase(x_series.begin());
        x_series.push_back(x_after);
        P = P_after;
        // 余分だがdebug用
        x = x_after;
    }

    // Input next_est(){
    //     Input next_u;
    //     next_u.v = u.v;
    //     next_u.w = u.w;
    //     return next_u;
    // }

    void output(){//状態の出力
        cout << "state is "<< x.transpose() << ", observable is " << y.transpose() << ", input is " 
        << u.transpose() << ", variation of state is " << P << endl;
    }

    Matrix2d Rotation2d(const double theta){
            Matrix2d R;
            R << cos(theta),-sin(theta),
                sin(theta),cos(theta);
            return R;
    }    
};

#endif

