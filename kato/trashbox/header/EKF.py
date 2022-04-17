import numpy as np

class Car():#privateなクラスの作り方が分からない
    def __init__(self,d_init=None,e_init=None):
        if d_init is None:
            self.d = np.array([[104.0,58.0],[-104.0,58.0],[0,-117.0]]).transpose()# [d1.T,d2.T,d3.T]
        if e_init is None:
            self.e = np.array([[-0.5,0.86],[-0.5,-0.86],[1.0,0.0]]).transpose()# [e1.T,e2.T,e3.T]
        self.a = 24
    

class EKF(Car):
    def __init__(self,d_init=None,e_init=None): # 諸々を書くのやめた
        super().__init__(d_init,e_init)
        # self.series_length = 5
        self.x = np.zeros((3,1))        
        # self.x_series = np.tile(np.array([[0],[0],[0]]),(1,self.series_length))
        self.t_diff = 0.1
        # self.F = np.eye(3)
        # self.H = np.zeros(3,3)
        # self.H[0,0] = 1
        # self.H[1,1] = 1
        # for i in range(3):
        #     self.H[i+2,2] = 1/self.e[:,i].transpose()*self.Rotation2d(np.pi/2)*self.d[:,i]
    
    def update_theta(self,omega,time_d):
        self.x[2] += omega*time_d
        self.x[2] = np.fmod(self.x[2]+2*np.pi,2*np.pi)
    
    def set_up_ekf(self,came):
        self.x[0] = came['x']
        self.x[1] = came['y']
        
    def obs_linear(self,theta_k):#実装＆デバッグを軽くするために線形変換を使います
        A = np.zeros((3,3))
        for i in range(3):
            A[i,0] = self.e[0,i]*np.cos(theta_k) - self.e[1,i]*np.sin(theta_k)
            A[i,1] = self.e[0,i]*np.sin(theta_k) + self.e[1,i]*np.cos(theta_k)
            A[i,2] = self.e[:,i]@self.Rotation2d(np.pi*0.5)@self.d[:,i].reshape(-1,1) # スライスすると行ベクトルになっちゃうので余分に転置がかかる
        return A/self.a
    
    def update(self,input_type,y_k):
        if input_type == 0: #encoder
            A = self.obs_linear(self.x[2])
            v_k = np.linalg.pinv(A)@y_k[2:].reshape(-1,1)
            self.x[0] += v_k[0]*self.t_diff
            self.x[1] += v_k[1]*self.t_diff
            self.update_theta(v_k[2],self.t_diff)
            # self.x[2] = 0
        elif input_type ==1: 
            self.x = y_k[:,0:3] 
            # v_kの補間は放置   
    
    def set_t_diff(self,time_d):
        self.t_diff = time_d        
    
    def Rotation2d(self,theta_k):
        return np.array([[np.cos(theta_k), -np.sin(theta_k)],
                  [np.sin(theta_k),  np.cos(theta_k)]]).reshape(2,2)
        
    def output(self):
        print(*self.x[:,0])
        
    def obs_evolution(self,u):
        omega = np.zeros(3)
        for i in range(3):
            omega[i] += self.e[:,i]@self.Rotation2d(-self.x[2])@u[0:2].transpose() 
            omega[i] += self.e[:,i]@self.Rotation2d(np.pi*0.5)@self.d[:,i].transpose()*u[2] 
        return omega/self.a
