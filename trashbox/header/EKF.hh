#ifndef EKF_CLASS__
#define EKF_CLASS__

#include "../Eigen/Dense"
#include "../others/dynamical_system.hh"

namespace Kalman
{
    class EKF:public dynamical_system
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

        State x;
        Input u;
        std::vector<State> x_series; // 状態の時系列 (2つはいっていればいい)
        State_Var P;//3*3行列
        std::vector<State_Var> P_series; // 共分散の時系列 (2つはいっていればいい)
        Car car;

    public:
        //初期化　construct
        EKF(){//とりあえず全部0で初期化しておきます。0以外を入れたい時はmain.cpp の　set_up()で入れることにしましょう。
            State x1;State_Var P1;
            x1 << 0, 0, 0;
            P1 << 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0;

            //y << 0, 0, 0, 0, 0, 0;
            x << 0, 0, 0;
            P << 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0;

            State zero_vec = Eigen::VectorXd::Zero(3);
            State_Var zero_mat = Eigen::MatrixXd::Zero(3,3);
            x_series.push_back(x1);
            P_series.push_back(P1);

            x_series.push_back(x);
            P_series.push_back(P);
            set_dt(0.01);
        }

        void set_u(Input uu){
            u = uu;
            return;
        }

        void show_u(){
            std::cout<< u <<std::endl;
            return;
        }

        void show_x(){
            std::cout<< x << std::endl;
            return;
        }
    
        void debug(){
            std::cout << "debuggin EKF" << std::endl;
            std::cout << "x:" << x.transpose() <<std::endl
            << "u:" << u.transpose() << std::endl <<"P:" << P << std::endl;//これで出力できるのつよすぎ(感嘆)
            /*
            for(State hoge:x_series){
                std::cout << "x-series are " << hoge.transpose() << std::endl;
            }
            for(State_Var fuga:P_series){
                std::cout << "P-series are " << fuga << std::endl;
            }
            */
            return;
        }

        void update(Encoder enc, Input_estimate est,Obs y_k){//1ステップ計算する
            set_u(est.get_input());

            //std::cout<<"u:";
            //show_u();
            //std::cout<<std::endl;
            // 時間更新式
            State x_middle = state_evolution(x,u);// x_k+1|k
            State_Var P_middle = evo_update_state_var(P,x,u);// P_k+1|k //引数不明

            // 観測更新
            Gain K = est_update_gain(P_middle);
            State x_new = est_update_state(x_middle,u,y_k,K);//x_k+1|k+1
            State_Var P_new = est_update_state_var(P_middle,K);//P_k+1|k+1

            // x,Pともに更新
            x_series.erase(x_series.begin());
            x_series.push_back(x_new);
            P_series.erase(P_series.begin());
            P_series.push_back(P_new);
            P = P_new;
            x = x_new;
        }

        void output(){//状態の出力
            std::cout<<x.transpose()<<std::endl;
        }

        std::vector<State> get_x_series(){ return x_series; }

        State state_evolution(const State &x_k, const Input &u_k){
            State x_next;
            double delta_t = get_dt();
            x_next(0) = x_k(0) + u_k(0) * cos(x(2))*delta_t;
            x_next(1) = x_k(1) + u_k(0) * sin(x(2))*delta_t;
            x_next(2) = x_k(2) + u_k(1) *delta_t;
            return x_next;
        }

        Obs observe_evolution(const State &x_k, const Input &u_k){
            Obs y_next;
            y_next(0) = x_k(0), y_next(1) = x_k(1), y_next(2)= u_k(1);
            for (int i = 0; i < 3; i++)
            {
                y_next(i+3) = enc_transformation(i,u);   
            }
            return y_next;
        }

        double enc_transformation(const int &i, const Input &u_k){
            double pos = car.e[i](0)*u_k(0);
            auto vel = (car.e[i].transpose())*Rotation2d(PI/2.0)*car.d[i]*u_k(0);
            return (pos + vel(0))/car.a;
        }
    
        matrix3d F_est(const State &x, const Input &u){
            matrix3d F = Eigen::MatrixXd::Zero(3, 3);
            double delta_t = get_dt();
            F(0,0) = 1;
            F(0,2) = -u(0)*sin(x(2))*delta_t ;
            F(1,1) = 1;
            F(1,2) = u(0)*cos(x(2))*delta_t ;
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

        State est_update_state(const State &x_k, const Input &u_k, const Obs &y_k,const Gain &K){
            return x_k + K * (y_k-observe_evolution(x_k,u_k));
        }

        State_Var est_update_state_var(const State_Var &P_before, const Gain &K){
            return P_before - K*H_est()*P_before;
        }

        State_Var evo_update_state_var(const State_Var &x_after, const State &x_k,const Input &u_k){
            matrix3d F = F_est(x_k,u_k);
            return F*x_after*F.transpose()+car.Xi;
        }


        matrix2d Rotation2d(const double theta_k){
            matrix2d R;
            R << cos(theta_k),-sin(theta_k),
                sin(theta_k),cos(theta_k);
            return R;
        }    
    };
}
#endif

