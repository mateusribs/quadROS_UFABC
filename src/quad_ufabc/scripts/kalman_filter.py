import pickle
import os, sys
import numpy as np
import scipy as sci

from numpy.linalg import inv, det
from scipy.spatial.transform import Rotation as Rot
from quat_utils import Quat2Rot, QuatProd, SkewMat, DerivQuat, Conj

#Set main directory
mydir = os.path.abspath(sys.path[0])


class KF():

    def __init__(self):


        # #Initial covariance matrix for MEKF
        # self.P_K = np.eye(6)*1000
        # #Initial error state vector for MEKF
        # self.ex_k = np.zeros((6,1))

        #Check if there is standard deviation data to import.
        if os.path.exists(mydir + '/' + 'data/std_sensors.p'):
            
            infile = open(mydir + '/' + 'data/std_sensors.p', 'rb')
            data = pickle.load(infile)
            infile.close()

            #Import standard deviation data
            std_accel = data['accel']
            std_gyro = data['gyro']
            std_cam_pos = data['cam pos']
            std_cam_att = data['cam att']

            print('---------Standard Deviation----------')
            print('Accel:', std_accel.T)
            print('Gyro:', std_gyro.T)
            print('Cam Pos:', std_cam_pos.T)
            print('Cam Att:', std_cam_att.T)
            print('-------------------------------------')

        #Set variances
        self.var_a = 0.05**2
        self.var_c_pos = np.max(std_cam_pos)**2
        self.var_c_att = np.max(std_cam_att)**2
        self.var_g = np.max(std_gyro)**2
        self.var_gb = (1e-4)**2

        #Intial quaternion
        self.q_K = np.array([[1, 0, 0, 0]]).T
        #Initial position
        self.pos_K = np.array([[0.6, 0, 0]]).T
        #Initial linear velocity
        self.v_K = np.array([[0, 0, 0]]).T
        #Initial gyroscope bias
        self.b_K = np.array([[0, 0, 0]]).T
        #Initial covariance matrix for ErEKF
        self.P_K = np.diag([1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3])
        #Initial error state vector for ErEKF
        self.ex_k = np.zeros((12,1))


    #Multiplicative Extended Kalman Filter
    def MEKF(self, accel, gyro, cam_vec):
        
        self.dt = 0.01
            
        #Predict Quaternion using Gyro Measuments
        self.q_k, self.b_k, self.P_k = self.Prediction_MEKF(self.q_K, self.P_K, self.b_K, gyro)


        self.q_K, self.b_K, self.P_K = self.Update_MEKF(self.ex_k, self.q_k, self.b_k, self.P_k, accel, cam_vec)        

        # print('Quaternion:', self.q_K.T)
        # print('Gyro Bias:', self.b_K.T)

        #Reset
        self.ex_k = np.zeros((6,1))

        #Trace
        self.trace = np.trace(self.P_K)
 
    def Prediction_MEKF(self, q_K, P_K, b_K, gyro):

        # Discrete Quaternion Propagation
        omega_hat_k = gyro - b_K
        omega_hat_k_norm = np.linalg.norm(omega_hat_k)

        omega_d = np.zeros([4,1])
        omega_d[0] = np.cos(omega_hat_k_norm*self.dt*0.5)
        omega_d[1:4] = (omega_hat_k/omega_hat_k_norm)*np.sin(omega_hat_k_norm*self.dt*0.5)

        q_kp1 =  QuatProd(q_K, omega_d)
        q_kp1 *= 1/(np.linalg.norm(q_kp1))

        b_kp1 = b_K

        #Discrete error-state transition matrix

        Phi_d = np.eye(6)
        Phi_d[0:3, 0:3] = np.eye(3) - SkewMat(omega_hat_k)*np.sin(omega_hat_k_norm*self.dt)/omega_hat_k_norm + SkewMat(omega_hat_k)@SkewMat(omega_hat_k)*(1 - np.cos(omega_hat_k_norm*self.dt))/(omega_hat_k_norm)**2
        Phi_d[0:3, 3:6] = SkewMat(omega_hat_k)*(1 - np.cos(omega_hat_k_norm*self.dt))/(omega_hat_k_norm)**2 - np.eye(3)*self.dt - SkewMat(omega_hat_k)@SkewMat(omega_hat_k)*(omega_hat_k_norm*self.dt - np.sin(omega_hat_k_norm*self.dt))/(omega_hat_k_norm)**3
        
        # input matrix

        G = np.eye(6)
        G[0:3, 0:3] = -G[0:3, 0:3]
        
        #Discrete noise process covariance matrix

        Q = np.eye(6)
        Q[0:3, 0:3] = np.eye(3)*self.var_g*self.dt**2
        Q[3:6, 3:6] = np.eye(3)*self.var_gb*self.dt

        #Discrete Covariance Propzagation
        P_kp1 = Phi_d@P_K@Phi_d.T + G@Q@G.T

        return q_kp1, b_kp1, P_kp1

    def Update_MEKF(self, dx_k, q_k, b_k, P_k, accel_norm, cam_vec):
        
        #Estimated Rotation Matrix - Global to Local - Inertial to Body
        A_q = Quat2Rot(q_k).T

        #Current Sensor Measurement

        #Accelerometer
        b_a = accel_norm

        if cam_vec is not None:
       
            #Camera
            b_c = cam_vec

            #Measurement Sensor Vector
            b = np.concatenate((b_a, b_c), axis=0)

            #Reference Accelerometer Direction (Gravitational Acceleration)
            r_a = np.array([[0, 0, 9.81]]).T
            #Reference Camera Direction
            r_c = np.array([[1, 0, 0]]).T


            #Camera Estimated Output
            b_hat_c = A_q @ r_c
            #Accelerometer Estimated Output
            b_hat_a = A_q @ r_a
            #Estimated Measurement Vector
            b_hat = np.concatenate((b_hat_a, b_hat_c), axis=0)

            #Sensitivy Matrix 
            H_k = np.zeros((6,6))
            H_k[0:3, 0:3] = SkewMat(b_hat_a)
            H_k[3:6, 0:3] = SkewMat(b_hat_c)
            
            
            #General Measurement Covariance Matrix
            R = np.eye(6)
            R[0:3, 0:3] = np.eye(3)*self.var_a
            R[3:6, 3:6] = np.eye(3)*self.var_c_att

            #Kalman Gain
            K_k = P_k@H_k.T @ inv(H_k@P_k@H_k.T + R)

            #Update Covariance
            P_K = (np.eye(6) - K_k@H_k)@P_k

            #Inovation (Residual)
            e_k = b - b_hat

            #Update State
            dx_k = K_k@e_k

        else:

            #Reference Accelerometer Direction (Gravitational Acceleration)
            r_a = np.array([[0, 0, 9.81]]).T
            #Accelerometer Estimated Output
            b_hat_a = A_q @ r_a

            #Sensitivy Matrix Accelerometer
            H_ka = np.zeros((3,6))
            H_ka[0:3, 0:3] = SkewMat(b_hat_a)

            #Accelerometer Measurement Covariance Matrix
            Ra = np.eye(3)*self.var_a

            #Kalman Gain
            K_k = P_k@H_ka.T @ inv(H_ka@P_k@H_ka.T + Ra)

            #Update Covariance
            P_K = (np.eye(6) - K_k@H_ka)@P_k

            #Inovation (Residual)
            e_k = b_a - b_hat_a
  
            #Update State
            dx_k = K_k@e_k
        
        #Update Quaternion
        dq_k = np.ones([4,1])
        dq_k[1:4] = 0.5*dx_k[0:3]
        q_K = QuatProd(q_k, dq_k)
        q_K *= 1/(np.linalg.norm(q_K))    

        #Update Biases
        db = dx_k[3:6]
        b_K = b_k + db

        return q_K, b_K, P_K




    #Error State Extended Kalman Filter

    def ErEKF(self, accel_raw, ang_vel, cam_att, cam_pos, cam2_att, cam2_pos, dt):
        
        #Sample rate
        self.dt = dt

        #Predict position and quaternion using IMU measurements
        self.pos_k, self.v_k, self.q_k, self.b_k, self.P_k = self.Predict_ErEKF(self.q_K, self.pos_K, self.v_K, self.b_K,self.P_K, accel_raw, ang_vel)

        if cam_att is not None or cam2_att is not None:

            #Update position and quaternion using accelerometer and camera measurements
            self.pos_K, self.v_K, self.q_K, self.b_K, self.P_K = self.Update_ErEKF_Total(self.ex_k, self.pos_k, self.v_k, self.q_k, self.b_k, self.P_k, accel_raw, cam_att, cam_pos, cam2_att, cam2_pos)
        
        else:
            
            #Update position and quaternion using accelerometer and camera measurements
            self.pos_K, self.v_K, self.q_K, self.b_K, self.P_K = self.Update_ErEKF_IMU(self.ex_k, self.pos_k, self.v_k, self.q_k, self.b_k, self.P_k, accel_raw)


        #Reset
        self.ex_k = np.zeros((12, 1))

        #Trace
        self.trace = np.trace(self.P_K)



    def Predict_ErEKF(self, q_K, p_K, v_K, b_K, P_K, accel_raw, gyro):
        
        R = Quat2Rot(q_K)
        g = np.array([[0, 0, -9.8]]).T

        accel = accel_raw 

        #Position Propagation
        p_kp1 = p_K + self.dt*v_K + 0.5*(self.dt**2)*(R@accel + g)
        v_kp1 = v_K + self.dt*(R@accel + g)


        # Discrete Quaternion Propagation
        omega_hat_k = gyro - b_K
        omega_hat_k_norm = np.linalg.norm(omega_hat_k)
        
        omega_d = np.zeros([4,1])
        omega_d[0] = np.cos(omega_hat_k_norm*self.dt*0.5)
        omega_d[1:4] = (omega_hat_k/omega_hat_k_norm)*np.sin(omega_hat_k_norm*self.dt*0.5)

        q_kp1 =  QuatProd(q_K, omega_d)
        q_kp1 *= 1/(np.linalg.norm(q_kp1))
        
        #Bias propagation

        b_kp1 = b_K

        # Discrete error-state transition matrix

        F = np.eye(12)
        #Error Position Dynamics
        F[0:3, 3:6] = np.eye(3)*self.dt
        #Error Velocity Dynamics
        F[3:6, 6:9] = -R@SkewMat(accel)*self.dt
        #Error Attitude Dynamics
        F[6:9, 6:9] = np.eye(3) - SkewMat(omega_hat_k)*np.sin(omega_hat_k_norm*self.dt)/omega_hat_k_norm + SkewMat(omega_hat_k)@SkewMat(omega_hat_k)*(1 - np.cos(omega_hat_k_norm*self.dt))/(omega_hat_k_norm)**2
        # F[6:9, 12:15] = SkewMat(omega_hat_k)*(1 - np.cos(omega_hat_k_norm*self.dt))/(omega_hat_k_norm)**2 - np.eye(3)*self.dt - SkewMat(omega_hat_k)@SkewMat(omega_hat_k)*(omega_hat_k_norm*self.dt - np.sin(omega_hat_k_norm*self.dt))/(omega_hat_k_norm)**3
        # F[6:9, 6:9] = np.eye(3) - SkewMat(omega_hat_k)*self.dt
        F[6:9, 9:12] = -np.eye(3)*self.dt


        #Discrete noise process covariance matrix

        Q = np.zeros((12,12))
        Q[3:6, 3:6] = np.eye(3)*self.var_a*self.dt**2
        Q[6:9, 6:9] = np.eye(3)*self.var_g*self.dt**2
        Q[9:12, 9:12] = np.eye(3)*self.var_gb*self.dt


        #Discrete Covariance Propagation
        P_kp1 = F@P_K@F.T + Q

        return p_kp1, v_kp1, q_kp1, b_kp1, P_kp1

    def Update_ErEKF_Total(self, dx_k, p_k, v_k, q_k, b_k, P_k, accel_raw, cam_att, cam_pos, cam2_att, cam2_pos):
        
        #Estimated Rotation Matrix - Global to Local - Inertial to Body
        A_q = Quat2Rot(q_k).T

        #Current Sensor Measurement

        #Accelerometer
        # accel_norm = np.linalg.norm(accel_raw - a_k)
        b_a = accel_raw 

        #Check if there is information from any camera
        if cam_att is not None or cam2_att is not None:
        
            
            #Information from both cameras
            if cam_att is not None and cam2_att is not None:

                #Cameras attitude vector
                b_c1 = cam_att
                b_c2 = cam2_att

                #Measurement Sensor Vector
                b = np.concatenate((cam_pos, cam2_pos, b_c1, b_c2, b_a), axis=0)

                #Reference Cameras Direction
                r_c1 = np.array([[1, 0, 0]]).T
                r_c2 = np.array([[1, 0, 0]]).T

                #Reference Accelerometer Direction (Gravitational Acceleration)
                r_a = np.array([[0, 0, 9.8]]).T

                #Cameras Estimated Output
                b_hat_c1 = A_q @ r_c1
                b_hat_c2 = A_q @ r_c2
                #Accelerometer Estimated Output
                b_hat_a = A_q @ r_a

                #Estimated Measurement Vector
                b_hat = np.concatenate((p_k, p_k, b_hat_c1, b_hat_c2, b_hat_a), axis=0)


                #Sensitivy Matrix 
                H_k = np.zeros((15, 12))
                H_k[0:3, 0:3] = np.eye(3)
                H_k[3:6, 0:3] = np.eye(3)
                H_k[6:9, 6:9] = SkewMat(b_hat_c1)
                H_k[9:12, 6:9] = SkewMat(b_hat_c2)
                H_k[12:15, 6:9] = SkewMat(b_hat_a)

                
                #General Measurement Covariance Matrix
                R = np.eye(15)
                R[0:3, 0:3] = R[0:3, 0:3]*self.var_c_pos
                R[3:6, 3:6] = R[3:6, 3:6]*self.var_c_pos
                R[6:9, 6:9] = R[6:9, 6:9]*self.var_c_att
                R[9:12, 9:12] = R[9:12, 9:12]*self.var_c_att
                R[12:15, 12:15] = R[12:15,12:15]*self.var_a

                #Kalman Gain
                K_k = P_k@H_k.T @ inv(H_k@P_k@H_k.T + R)

                #Update Covariance
                P_K = (np.eye(12) - K_k@H_k)@P_k

                #Inovation (Residual)
                self.e_k = b - b_hat

                #Update State
                dx_k = K_k@self.e_k

            #Only camera 1 information
            if cam_att is not None and cam2_att is None:

                #Camera 1 attitude vector
                b_c1 = cam_att

                #Measurement Sensor Vector
                b = np.concatenate((cam_pos, b_c1, b_a), axis=0)

                #Reference Camera Direction
                r_c1 = np.array([[1, 0, 0]]).T

                #Reference Accelerometer Direction (Gravitational Acceleration)
                r_a = np.array([[0, 0, 9.8]]).T


                #Camera Estimated Output
                b_hat_c1 = A_q @ r_c1
                #Accelerometer Estimated Output
                b_hat_a = A_q @ r_a
                #Estimated Measurement Vector
                b_hat = np.concatenate((p_k, b_hat_c1, b_hat_a), axis=0)


                #Sensitivy Matrix 
                H_k = np.zeros((9, 12))
                H_k[0:3, 0:3] = np.eye(3)
                H_k[3:6, 6:9] = SkewMat(b_hat_c1)
                H_k[6:9, 6:9] = SkewMat(b_hat_a)

                
                #General Measurement Covariance Matrix
                R = np.eye(9)
                R[0:3, 0:3] = R[0:3, 0:3]*self.var_c_pos
                R[3:6, 3:6] = R[3:6, 3:6]*self.var_c_att
                R[6:9, 6:9] = R[6:9, 6:9]*self.var_a

                #Kalman Gain
                K_k = P_k@H_k.T @ inv(H_k@P_k@H_k.T + R)

                #Update Covariance
                P_K = (np.eye(12) - K_k@H_k)@P_k

                #Inovation (Residual)
                self.e_k = b - b_hat

                #Update State
                dx_k = K_k@self.e_k


            #Only camera 2 information
            if cam_att is None and cam2_att is not None:

                #Camera 2 attitude vector
                b_c2 = cam2_att

                #Measurement Sensor Vector
                b = np.concatenate((cam2_pos, b_c2, b_a), axis=0)

                #Reference Camera Direction
                r_c2 = np.array([[1, 0, 0]]).T

                #Reference Accelerometer Direction (Gravitational Acceleration)
                r_a = np.array([[0, 0, 9.8]]).T

                #Camera Estimated Output
                b_hat_c2 = A_q @ r_c2
                #Accelerometer Estimated Output
                b_hat_a = A_q @ r_a
                #Estimated Measurement Vector
                b_hat = np.concatenate((p_k, b_hat_c2, b_hat_a), axis=0)


                #Sensitivy Matrix 
                H_k = np.zeros((9, 12))
                H_k[0:3, 0:3] = np.eye(3)
                H_k[3:6, 6:9] = SkewMat(b_hat_c2)
                H_k[6:9, 6:9] = SkewMat(b_hat_a)

                
                #General Measurement Covariance Matrix
                R = np.eye(9)
                R[0:3, 0:3] = R[0:3, 0:3]*self.var_c_pos
                R[3:6, 3:6] = R[3:6, 3:6]*self.var_c_att
                R[6:9, 6:9] = R[6:9, 6:9]*self.var_a

                #Kalman Gain
                K_k = P_k@H_k.T @ inv(H_k@P_k@H_k.T + R)

                #Update Covariance
                P_K = (np.eye(12) - K_k@H_k)@P_k

                #Inovation (Residual)
                self.e_k = b - b_hat

                #Update State
                dx_k = K_k@self.e_k
                
        self.dx_k = dx_k

        #Update Quaternion
        dq_k = np.ones([4,1])
        dq_k[1:4] = 0.5*dx_k[6:9]
        q_K = QuatProd(q_k, dq_k)
        q_K *= 1/(np.linalg.norm(q_K))     

        #Update Position
        dp = dx_k[0:3, :]
        p_K = p_k + dp

        #Update Velocity
        dv = dx_k[3:6, :]
        v_K = v_k + dv

        #Update Biases
        db = dx_k[9:12, :]
        b_K = b_k + db


        return p_K, v_K, q_K, b_K, P_K

    def Update_ErEKF_IMU(self, dx_k, p_k, v_k, q_k, b_k, P_k, accel_raw):

        #Estimated Rotation Matrix - Global to Local - Inertial to Body
        A_q = Quat2Rot(q_k).T


        #Current Sensor Measurement

        #Accelerometer
        # accel_norm = np.linalg.norm(accel_raw - a_k)
        b_a = accel_raw

        #IMU Measurement Sensor Vector
        b = b_a

        #Reference Accelerometer Direction (Gravitational Acceleration)
        r_a = np.array([[0, 0, 9.8]]).T
        #Accelerometer Estimated Output
        b_hat_a = A_q @ r_a

        #Estimated Measurement Vector
        b_hat = b_hat_a

        #Sensitivy Matrix 
        H_k = np.zeros((3,12))
        H_k[0:3, 6:9] = SkewMat(b_hat_a)
        
        #General Measurement Covariance Matrix
        R = np.eye(3)
        R[0:3, 0:3] = np.eye(3)*self.var_a

        #Kalman Gain
        K_k = P_k@H_k.T @ inv(H_k@P_k@H_k.T + R)

        #Update Covariance
        P_K = (np.eye(12) - K_k@H_k)@P_k

        #Inovation (Residual)
        self.e_k = b - b_hat

        #Update State
        dx_k = K_k@self.e_k
        

        self.dx_k = dx_k

        #Update Quaternion
        dq_k = np.ones([4,1])
        dq_k[1:4] = 0.5*dx_k[6:9]
        q_K = QuatProd(q_k, dq_k)
        q_K *= 1/(np.linalg.norm(q_K))     

        #Update Position
        dp = dx_k[0:3, :]
        p_K = p_k + dp

        #Update Velocity
        dv = dx_k[3:6, :]
        v_K = v_k + dv

        #Update Biases
        db = dx_k[9:12, :]
        b_K = b_k + db

        return p_K, v_K, q_K, b_K, P_K

