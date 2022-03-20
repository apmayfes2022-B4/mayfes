#ifndef EKF_CLASS__
#define EKF_CLASS__

#include "../Eigen/Dense"
#include "../others/dynamical_system.hh"

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

namespace Kalman
{
    class EKF:dynamical_system
    {
    private:
    //車のモデルはここ
        class Car
        {
        public:
            vector2d d[3]; //モータの位置ベクトル
            vector2d e[3]; //モータの進行方向ベクトル

            matrix3d Xi;
            matrix6d Eta;    //ノイズの共分散  
            double a; //回転半径
        
            Car(){//初期化
                a = 1;
                vector2d d1(1.0,0);
                vector2d d2(-1.0,0);
                vector2d d3(0,1.71);
                d[0] = d1;d[1] = d2;d[2] = d3;
            
                vector2d e1(0.86,0.5);  //(sqrt{3}/2, 1/2)
                vector2d e2(-0.86,-0.5);
                vector2d e3(0.5,-0.86);
                e[0] = e1;e[1] = e2;e[2] = e3;
                matrix3d xi = Eigen::MatrixXd::Identity(3,3);
                Xi = xi;
                matrix6d eta = Eigen::MatrixXd::Identity(6,6);
                Eta = eta;
            }
        };

    public:
        Obs y;
        State x;
        Input u;
        std::vector<State> x_series; // 状態の時系列 
        State_Var P;//3*3行列
        std::vector<State_Var> P_series; // 共分散の時系列 
        Car car;
        
        //初期化　construct
        EKF(){//とりあえず全部0で初期化しておきます。0以外を入れたい時はmain.cpp の　set_up()で入れることにしましょう。
            y << 0, 0, 0, 0, 0, 0;
            x << 0, 0, 0;
            P << 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0;
            State zero_vec = Eigen::VectorXd::Zero(3);
            State_Var zero_mat = Eigen::MatrixXd::Zero(3,3);
            x_series.push_back(x);
            P_series.push_back(P);
            //あとでエラーを吐かないようにするため//条件分岐よりはよいかな。
            x_series.push_back(x);
            P_series.push_back(P);
            dt = 0.01;
        }
    
        void debug(){
            std::cout << "debuggin EKF" << std::endl;
            std::cout << "y is " << y.transpose() << "x is " << x.transpose() 
            << "u is " << u.transpose() << "P is " << P << std::endl;//これで出力できるのつよすぎ(感嘆)
            for(State hoge:x_series){
                std::cout << "x-series are " << hoge.transpose() << std::endl;
            }
            for(State_Var fuga:P_series){
                std::cout << "P-series are " << fuga << std::endl;
            }
            return;
        }

        void update(Encoder enc, Input_estimate est,Obs y_k){//1ステップ計算する
            //auto の使用はできるだけ控えて欲しい　型がわからなくて読みづらいので
            u = est.u;
            // 時間更新式
            State x_middle = state_evolution(x,u);// x_k+1|k
            State_Var P_middle = evo_update_state_var(P,x,u);// P_k+1|k //引数不明

            // 観測更新
            Gain K = est_update_gain(P_middle);
            State x_new = est_update_state(x_middle,u,y_k,K);//x_k+1|k+1
            State_Var P_new = est_update_state_var(P_middle,K);//P_k+1|k+1
        
            // x[4],Pともに更新
            x_series.erase(x_series.begin());
            x_series.push_back(x_new);
            P_series.erase(P_series.begin());
            P_series.push_back(P_new);
            P = P_new;
            x = x_new;
        }

        void output(){//状態の出力
            std::cout<<x.transpose()<<std::endl;
            //cout << "state is "<< x.transpose() << ", observable is " << y.transpose() <<endl;//<< ", input is " 
            //<< u.transpose() << ", variation of state is " << P << endl;
        }

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
            double pos = car.e[i](0)*u(0);
            auto vel = (car.e[i].transpose())*Rotation2d(PI/2.0)*car.d[i]*u(0);
            return (pos + vel(0))/car.a;
        }
    
        matrix3d F_est(const State &x, const Input &u){
            matrix3d F = Eigen::MatrixXd::Zero(3, 3);
            F(0,0) = 1;
            F(0,2) = -u(0)*sin(x(2))*dt ;
            F(1,1) = 1;
            F(1,2) = u(0)*cos(x(2))*dt ;
            F(2,2) = 1;
            return F;
        }

        Eigen::Matrix<double,6,3> H_est(void){
            Eigen::Matrix<double,6,3> H = Eigen::MatrixXd::Zero(6, 3);
            H(0,0) = 1;
            H(1,1) = 1;
            return H;
        }

        Gain est_update_gain(const State_Var &P_before){
            matrix6d content_of_Inv = H_est()*P_before*H_est().transpose()+car.Eta;
            return P_before*H_est().transpose()*(content_of_Inv).inverse();
        }

        State est_update_state(const State &x, const Input &u, const Obs &y,const Gain &K){
            return x + K * (y-observe_evolution(x,u));
        }

        State_Var est_update_state_var(const State_Var &P_before, const Gain &K){
            return P_before - K*H_est()*P_before;
        }

        State_Var evo_update_state_var(const State_Var &x_after, const State &x,const Input &u){
            matrix3d F = F_est(x,u);
            return F*x_after*F.transpose()+car.Xi;
        }


        matrix2d Rotation2d(const double theta){
            matrix2d R;
            R << cos(theta),-sin(theta),
                sin(theta),cos(theta);
            return R;
        }    
    };
}
#endif

