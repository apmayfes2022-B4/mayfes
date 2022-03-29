#ifndef EKF_CLASS__
#define EKF_CLASS__

#include "../Eigen/Dense"

namespace Kalman
{
    class EKF
    {
    private:
        class Car
        {
        public:
            vector2d d[3]; //モータの位置ベクトル
            vector2d e[3]; //モータの進行方向ベクトル
            double theta;
            matrix3d Xi;
            matrix5d Eta;    //ノイズの共分散  
            double a; //回転半径
        
            Car(){//初期化
                theta = 0;
                a = 2;
                vector2d d1(1.0,0);
                vector2d d2(-1.0,0);
                vector2d d3(0,1.71);
                d[0] = d1;d[1] = d2;d[2] = d3;
            
                vector2d e1(0.86,0.5);  //(sqrt{3}/2, 1/2)
                vector2d e2(-0.86,-0.5);
                vector2d e3(0.5,-0.86);
                e[0] = e1;e[1] = e2;e[2] = e3;
                Xi = Eigen::MatrixXd::Identity(3,3);
                Eta = Eigen::MatrixXd::Identity(5,5);
            }

            void update_theta(double omega,double time_d){
                theta += omega*time_d;
                theta = fmod(theta,2*PI);
                return;
            }
        };
        std::vector<State> x_series; // 状態の時系列 (2つはいっていればいい)
        State_Var P;//3*3行列
        Car car;
        double t_diff;
        matrix3d F;
        Eigen::Matrix<double,5,3> H;


    public:
        //初期化　construct
        EKF(){//とりあえず全部0で初期化しておきます。0以外を入れたい時はmain.cpp の　set_up()で入れることにしましょう。
            State x1,x2,x3;
            x1 << 0, 0, 0;
            x2 << 0, 0, 0;
            x3 << 0, 0, 0;
            x_series.push_back(x1);
            x_series.push_back(x2);
            x_series.push_back(x3);
            P = Eigen::MatrixXd::Zero(3,3);
            F = Eigen::MatrixXd::Identity(3,3);
            H = Eigen::MatrixXd::Zero(5, 3);
            H(0,0) = 1;
            H(1,1) = 1;
            H(2,2) = 1/car.a*car.e[0].transpose()*Rotation2d(PI/2)*car.d[0];
            H(3,2) = 1/car.a*car.e[1].transpose()*Rotation2d(PI/2)*car.d[1];
            H(4,2) = 1/car.a*car.e[2].transpose()*Rotation2d(PI/2)*car.d[2];
            t_diff = 0.1;
        }

        void show_x(){
            std::cout<< x_series[2] << std::endl;
            return;
        }

        void show_P(){
            std::cout<< P << std::endl;
            return;
        }
    
        void debug(){
            std::cout << "debuging EKF" << std::endl;
            std::cout << "x:" << x_series[2].transpose() <<std::endl
            <<"P:" << P << std::endl;//これで出力できるのつよすぎ(感嘆)
            return;
        }

        void set_t_diff(double time_d){
            t_diff = time_d;
            car.update_theta(x_series[2](2),time_d);
            return;
        }

        void update(double input_type, Obs y_k){//1ステップ計算する
            // 時間更新式
            State x_middle = state_evolution();// x_k+1|k
            State_Var P_middle = evo_update_state_var();// P_k+1|k //引数不明
            if(input_type==0){//encorder
                y_k(0) = x_middle(0);
                y_k(1) = x_middle(1);
            }else if(input_type==1){
                y_k = make_encoder_omega(y_k);
            }
            // 観測更新
            Gain K = est_update_gain(P_middle);
            State x_new = est_update_state(x_middle,K,y_k);//x_k+1|k+1
            P = est_update_state_var(P_middle,K);//P_k+1|k+1
            // x,P,wともに更新
            x_series.erase(x_series.begin());
            x_series.push_back(x_new);
            return;
        }

        void output(){//状態の出力
            std::cout<<x_series[2](0)<<" "<<x_series[2](1)<<" "<<car.theta<<std::endl;
        }

        std::vector<State> get_x_series(){ return x_series; }

        State state_evolution(){
            State x_now = x_series[2];
            State x_before = x_series[1];
            State x_est;
            x_est(0) = 2*x_now(0) - x_before(0);
            x_est(1) = 2*x_now(1) - x_before(1);
            x_est(2) = 2*x_now(2) - x_before(2);
            return x_est;
        }

        Obs observe_evolution(const State &x_middle){
            Obs y_next;
            y_next(0) = x_middle(0), y_next(1) = x_middle(1);
            double omega = x_middle(2); 
            vector2d v_l;
            v_l(0) = (x_series[2](0) - x_series[1](0))/t_diff;
            v_l(1) = (x_series[2](1) - x_series[1](1))/t_diff;
            for (int i = 0; i < 3; i++)
            {
                y_next(i+2) = 1/car.a*(car.e[i].transpose()*Rotation2d(-car.theta)*v_l+double(car.e[i].transpose()*Rotation2d(PI/2)*car.d[i])*omega);   
            }
            return y_next;
        }

        Obs make_encoder_omega(const Obs &y_k){
            Obs y_made;
            vector2d v_l;
            v_l(0) = (x_series[1](0)-x_series[2](0))/t_diff;
            v_l(1) = (x_series[1](1)-x_series[2](1))/t_diff;
            double omega = x_series[1](2)-x_series[2](2);
            
            y_made(0) = y_k(0);
            y_made(1) = y_k(1);
            y_made(2) = 1/car.a*(car.e[0].transpose()*Rotation2d(-car.theta)*v_l+double(car.e[0].transpose()*Rotation2d(PI/2)*car.d[0])*omega);
            y_made(3) = 1/car.a*(car.e[1].transpose()*Rotation2d(-car.theta)*v_l+double(car.e[1].transpose()*Rotation2d(PI/2)*car.d[1])*omega);
            y_made(4) = 1/car.a*(car.e[2].transpose()*Rotation2d(-car.theta)*v_l+double(car.e[2].transpose()*Rotation2d(PI/2)*car.d[2])*omega);
            return y_made;
        }

        Gain est_update_gain(const State_Var &P_before){
            matrix5d content_of_Inv = H*P_before*H.transpose()+car.Eta;
            return P_before*H.transpose()*(content_of_Inv).inverse();
        }

        State est_update_state(const State x_middle,const Gain &K, const Obs &y_k){
            return x_middle + K * (y_k-observe_evolution(x_middle));
        }

        State_Var est_update_state_var(const State_Var &P_before, const Gain &K){
            return P_before - K*H*P_before;
        }

        State_Var evo_update_state_var(){
            return F*P*F.transpose()+car.Xi;
        }

        matrix2d Rotation2d(const double &theta_k){
            matrix2d R;
            R << cos(theta_k),-sin(theta_k),
                sin(theta_k),cos(theta_k);
            return R;
        }    
    };
}
#endif

