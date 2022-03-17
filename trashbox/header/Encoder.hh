#ifndef ENCODER_CLASS__
#define ENCODER_CLASS__

#define initial 0 //初期値
double dt=0.01;


class Encoder
{
public:
    int x[2],y[2],z[2];

    Encoder(int x_0 = 0,int y_0 = 0,int z_0 = 0){
        x[1] = x_0;
        y[1] = y_0;
        z[1] = z_0;
    }

    encoder Omega(){
        encoder omega[3]; 

        omega[0] = (x[1]-x[0])/dt;
        omega[1] = (y[1]-y[0])/dt;
        omega[2] = (z[1]-z[0])/dt;
        return 0;
    }

    void debug(){
        cout<<"Encoder"<<endl;
    }
};
#endif