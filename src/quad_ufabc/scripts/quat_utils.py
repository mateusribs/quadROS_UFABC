import numpy as np
import math

def QuatProd(p, q):
    
    pv = np.array([p[1], p[2], p[3]], dtype='float32')
    ps = p[0]

    qv = np.array([q[1], q[2], q[3]], dtype='float32')
    qs = q[0]

    scalar = ps*qs - pv.T@qv
    vector = ps*qv + qs*pv + np.cross(pv, qv, axis=0)

    q_res = np.concatenate((scalar, vector), axis=0)

    return q_res
    

def Quat2Rot(q):

    qv = np.array([q[1], q[2], q[3]], dtype='float32')
    qs = q[0]
    
    qvx = np.array([[0, -qv[2], qv[1]],
                    [qv[2], 0, -qv[0]], 
                    [-qv[1], qv[0], 0]], dtype='float32') 

    A_q = (qs**2 - qv.T@qv)*np.eye(3) + 2*qv@qv.T + 2*qs*qvx

    return A_q

def SkewMat(q):

    qx = q[0]
    qy = q[1]
    qz = q[2]

    omega = np.array([[0, -qz, qy],
                      [qz, 0, -qx],
                      [-qy, qx, 0]], dtype='float32')
    return omega

def DerivQuat(w, q):

    wx = w[0]
    wy = w[1]
    wz = w[2]   
    omega = np.array([[0, -wx, -wy, -wz],
                      [wx, 0, wz, -wy],
                      [wy, -wz, 0, wx],
                      [wz, wy, -wx, 0]], dtype='float32')
    dq = 0.5*omega@q

    return dq

def Euler2Quat(euler):

    cos_ph = np.cos(euler[0]*0.5)
    sin_ph = np.sin(euler[0]*0.5)

    cos_th = np.cos(euler[1]*0.5)
    sin_th = np.sin(euler[1]*0.5)

    cos_ps = np.cos(euler[2]*0.5)
    sin_ps = np.sin(euler[2]*0.5)

    q0 = cos_ph*cos_th*cos_ps + sin_ph*sin_th*sin_ps

    q1 = sin_ph*cos_th*cos_ps - cos_ph*sin_th*sin_ps

    q2 = cos_ph*sin_th*cos_ps + sin_ph*cos_th*sin_ps 

    q3 = cos_ph*cos_th*sin_ps - sin_ph*sin_th*cos_ps

    q = np.array([[q0, q1, q2, q3]]).T

    return q


def Conj(q):

    q1_c = -q[0]
    q2_c = -q[1]
    q3_c = -q[2]

    q_c = np.array([q1_c, q2_c, q3_c, q[3]])

    return q_c