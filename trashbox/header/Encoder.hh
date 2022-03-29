#ifndef ENCODER_CLASS__
#define ENCODER_CLASS__


namespace Kalman
{
    typedef struct{
        int x,y,z;
    }Scale;

    typedef struct{
        double x,y;
    }Camera;

    class Encoder
    {
    private:
    typedef struct{
        int x[2];
        int y[2];
        int z[2];
    }Time_series;

        Time_series time_series;
        double trans_coefficient;//エンコーダ目盛りを回転角度に翻訳
        double t_diff;

    public:
        Encoder(){
            time_series.x[0] = 0;
            time_series.x[1] = 0;
            time_series.y[0] = 0;
            time_series.y[1] = 0;
            time_series.z[0] = 0;
            time_series.z[1] = 0;
            t_diff = 0.1;
            trans_coefficient = 1/10000.0;
        }

        void set_t_diff(double time_d){
            t_diff = time_d;
            return;
        }

        std::vector<double> Omega(){//角速度の計算
            std::vector<double> omega(3); 
            omega[0] = (time_series.x[1]-time_series.x[0])/t_diff*trans_coefficient;
            omega[1] = (time_series.y[1]-time_series.y[0])/t_diff*trans_coefficient;
            omega[2] = (time_series.z[1]-time_series.z[0])/t_diff*trans_coefficient;
            return omega;
        }

        void show_Omega(){
            double delta_t = t_diff;
            std::cout<<"omega[0] = "<<(time_series.x[1]-time_series.x[0])/delta_t*trans_coefficient<<"omega[1] = "<<(time_series.y[1]-time_series.y[0])/delta_t*trans_coefficient<<"omega[2] = "<<(time_series.z[1]-time_series.z[0])/delta_t*trans_coefficient<<std::endl;
            return;
        }

        void update(Scale ee){
            // time_seriesの更新
            time_series.x[0] = time_series.x[1];
            time_series.y[0] = time_series.y[1];
            time_series.z[0] = time_series.z[1];
            time_series.x[1] = ee.x;
            time_series.y[1] = ee.y;
            time_series.z[1] = ee.z;
            return;
        }

        void debug(){
            std::cout<<"previously, x: "<<time_series.x[0]<<", y: "<<time_series.y[0]<<", z: "<<time_series.z[0]<<std::endl;
            std::cout<<"x: "<<time_series.x[1]<<", y: "<<time_series.y[1]<<", z: "<<time_series.z[1]<<std::endl;
            return ;
        }
    };
}
#endif