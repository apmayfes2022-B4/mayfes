import numpy as np
from header import EKF
from header import Encoder
        
def main():
    enc = Encoder.Encoder()
    ekf = EKF.EKF()
    go_on = True
    awake = False
    ee = dict(x=0,y=0,z=0)
    came = dict(x=0,y=0)
    input_type = 0
    t_now = 0
    t_before = 0
    t_diff = 0.01
    while go_on == True:
        input_data = [float(e) for e in input().split()]
        input_type = input_data[0]
        t_before = t_now
        t_now += 0.01
        t_diff = t_now -t_before
        if input_type==0:
            if not len(input_data)==4:
                raise RuntimeError('input_type is 0, but input_data size is NOT 3')
            else:
              ee['x'] = input_data[1]  
              ee['y'] = input_data[2]  
              ee['z'] = input_data[3]
            enc.set_t_diff(t_diff)
            enc.update(ee)
            omega = enc.Omega()
            ekf.set_t_diff(t_diff)
            y_now = np.vstack((np.array([[came['x']],[came['y']]]),omega))
            ekf.update(input_type,y_now)
        elif input_type == 1:
            if not len(input_data)==3:
                raise RuntimeError('input_type is 1, but input_data size is NOT 3')
            else:
              came['x'] = input_data[1]  
              came['y'] = input_data[2]  

              ekf.set_up_ekf(came)
            #if awake is False:
            #    awake = True
            #    ekf.set_up_ekf(came)
            
            # ekf.set_t_diff()
            # y_now = np.vstack(np.array([[came['x']],[came['y']]]),omega)
            # ekf.update(input_type,y_now)
        else:
            go_on = False

        ekf.output()

if __name__ == "__main__":
    main()