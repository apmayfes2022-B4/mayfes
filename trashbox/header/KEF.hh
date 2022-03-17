#ifndef EKF_CLASS__
#define EKF_CLASS__

#define State double
#define Obs double
#define Input double

#define encoder int

class EKF
{
    public:
        int x_,y_,w_j;
        encoder w[3];
        State x[3];

        EKF(int x_0 = 0,int y_0 = 0,int w_0 = 0){
            x_ = x_0;
            y_ = y_0;
            w_j = w_0;
        }

        void debug(){
            cout<<"Hello"<<endl;
        }
        
        State state_evolution(State x,Input u){
            State x_next = x;
            return x_next;
        }

        Obs observe_evolution(State x,Input u){
            Obs y = x;
            return y;
        }
        
};
#endif

