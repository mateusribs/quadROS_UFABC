#!/usr/bin/env python3
import math
import pickle
import os, sys
import statistics as stat
import numpy as np
import matplotlib.pyplot as plt


from sklearn.metrics import mean_squared_error
from scipy.spatial.transform import Rotation as R

plt.rcParams.update({'font.size': 15})

#Set main directory
mydir = os.path.abspath(sys.path[0])

#Results directory
results = 'results/Controle/Case 3/case_3.p'
case = 'case_3'

########################## PLOTTING ANALYSIS #############################################

if os.path.exists(mydir + '/' + results):
            
    infile = open(mydir + '/' + results, 'rb')
    states = pickle.load(infile)
    infile.close()

    real_pos = states['real position']
    des_pos = states['desired position']
    cam1_pos = states['measured position camera 1']
    cam2_pos = states['measured position camera 2']
    est_pos = states['estimated position']
    real_att = states['real attitude']
    cam1_att = states['measured attitude camera 1']
    cam2_att = states['measured attitude camera 2']
    des_att = states['desired attitude']
    est_att = states['estimated attitude']
    est_bias_g = states['estimated gyro bias']
    trace_list = states['trace'] 
    accel_list = states['accel meas']
    P_list = states['P']
    error_list = states['error state']
    vel_list = states['vel']
    vel_real_list = states['vel_real']

    

time = np.arange(0, 20, 0.01)

#Plot position
fig_pos, (x, y, z) = plt.subplots(3, 1, figsize=(14,8), sharex=True)

x.plot(time, des_pos[0], 'g--', label=r'$X_{des}(t)$')
y.plot(time, des_pos[1], 'g--', label=r'$Y_{des}(t)$')
z.plot(time, des_pos[2], 'g--', label=r'$Z_{des}(t)$')

x.plot(time, real_pos[0], 'r-.', label=r'$X_{real}(t)$')
y.plot(time, real_pos[1], 'r-.', label=r'$Y_{real}(t)$')
z.plot(time, real_pos[2], 'r-.', label=r'$Z_{real}(t)$')

x.plot(time, est_pos[0], 'b', label=r'$X_{est}(t)$')
y.plot(time, est_pos[1], 'b', label=r'$Y_{est}(t)$')
z.plot(time, est_pos[2], 'b', label=r'$Z_{est}(t)$')

# x.plot(time, cam1_pos[0], 'y', label=r'$X_{cam_1}(t)$', alpha=0.7)
# y.plot(time, cam1_pos[1], 'y', label=r'$Y_{cam_1}(t)$', alpha=0.7)
# z.plot(time, cam1_pos[2], 'y', label=r'$Z_{cam_1}(t)$', alpha=0.7)

# x.plot(time, cam2_pos[0], 'k', label=r'$X_{cam2}(t)$', alpha=0.7)
# y.plot(time, cam2_pos[1], 'k', label=r'$Y_{cam2}(t)$', alpha=0.7)
# z.plot(time, cam2_pos[2], 'k', label=r'$Z_{cam_2}(t)$', alpha=0.7)

z.set_xlabel('Tempo(s)')

x.set_ylabel(r'$X (m)$')
y.set_ylabel(r'$Y (m)$')
z.set_ylabel(r'$Z (m)$')

x.legend(bbox_to_anchor=(1.04,1), loc="upper left")
y.legend(bbox_to_anchor=(1.04,1), loc="upper left")
z.legend(bbox_to_anchor=(1.04,1), loc="upper left")

x.grid()
y.grid()
z.grid()


################################################################################################################

#Plot attitude in quaternion
fig_quat, (q0, q1, q2, q3) = plt.subplots(4, 1, figsize=(14,8), sharex=True)
q0.plot(time, real_att[0], 'r-.',  label=r'$q_{0,real}(t)$')
q1.plot(time, real_att[1], 'r-.', label=r'$q_{1,real}(t)$')
q2.plot(time, real_att[2], 'r-.', label=r'$q_{2,real}(t)$')
q3.plot(time, real_att[3], 'r-.', label=r'$q_{3,real}(t)$')

q0.plot(time, des_att[0], 'g--', label=r'$q0_{des}(t)$')
q1.plot(time, des_att[1], 'g--', label=r'$q1_{des}(t)$')
q2.plot(time, des_att[2], 'g--', label=r'$q2_{des}(t)$')
q3.plot(time, des_att[3], 'g--', label=r'$q3_{des}(t)$')

q0.plot(time, est_att[0], 'b', label=r'$q_{0,est}(t)$')
q1.plot(time, est_att[1], 'b', label=r'$q_{1,est}(t)$')
q2.plot(time, est_att[2], 'b', label=r'$q_{2,est}(t)$')
q3.plot(time, est_att[3], 'b', label=r'$q_{3,est}(t)$')

# q0.plot(time, cam1_att[0], 'y--', label=r'$q_{0,cam_1}(t)$', alpha=0.7)
# q1.plot(time, cam1_att[1], 'y--', label=r'$q_{1,cam_1}(t)$', alpha=0.7)
# q2.plot(time, cam1_att[2], 'y--', label=r'$q_{2,cam_1}(t)$', alpha=0.7)
# q3.plot(time, cam1_att[3], 'y--', label=r'$q_{3,cam_1}(t)$', alpha=0.7)

# q0.plot(time, cam2_att[0], 'k--', label=r'$q0_{cam_2}(t)$', alpha=0.7)
# q1.plot(time, cam2_att[1], 'k--', label=r'$q1_{cam_2}(t)$', alpha=0.7)
# q2.plot(time, cam2_att[2], 'k--', label=r'$q2_{cam_2}(t)$', alpha=0.7)
# q3.plot(time, cam2_att[3], 'k--', label=r'$q3_{cam_2}(t)$', alpha=0.7)

q3.set_xlabel('Tempo(s)')

q0.set_ylabel(r'$q_0$')
q1.set_ylabel(r'$q_1$')
q2.set_ylabel(r'$q_2$')
q3.set_ylabel(r'$q_3$')

q0.set_ylim(0.8, 1)
q1.set_ylim(-.04, .04)
q2.set_ylim(-.05, .05)
q3.set_ylim(-0.05, 0.4)

q0.legend(bbox_to_anchor=(1.04,1), loc="upper left")
q1.legend(bbox_to_anchor=(1.04,1), loc="upper left")
q2.legend(bbox_to_anchor=(1.04,1), loc="upper left")
q3.legend(bbox_to_anchor=(1.04,1), loc="upper left")

q0.grid()
q1.grid()
q2.grid()
q3.grid()

# # Plot attitude in Euler angles
cam1_phi, cam2_phi, real_phi, des_phi, est_phi = [], [], [], [], []
cam1_theta, cam2_theta, real_theta, des_theta, est_theta = [], [], [], [], []
cam1_psi, cam2_psi, real_psi, des_psi, est_psi = [], [], [], [], []


quaternions_list = [real_att, des_att, est_att, cam1_att, cam2_att]
phi_list = [real_phi, des_phi, est_phi, cam1_phi, cam2_phi]
theta_list = [real_theta, des_theta, est_theta, cam1_theta, cam2_theta]
psi_list = [real_psi, des_psi, est_psi, cam1_psi, cam2_psi]

cont = 0

for quat in quaternions_list:

    for k in range(0, len(quat[0])):

        r = R.from_quat([quat[1][k], quat[2][k], quat[3][k], quat[0][k]])
        euler_rad = r.as_euler('XYZ', degrees=False)

        phi_list[cont].append(euler_rad[0])
        theta_list[cont].append(euler_rad[1])
        psi_list[cont].append(euler_rad[2])

    cont += 1

fig_euler, (phi, theta, psi) = plt.subplots(3, 1, figsize=(14,8), sharex=True)

phi.plot(time, phi_list[0], 'r-.',  label=r'$\phi_{real}(t)$')
theta.plot(time, theta_list[0], 'r-.', label=r'$\theta_{real}(t)$')
psi.plot(time, psi_list[0], 'r-.', label=r'$\psi_{real}(t)$')

phi.plot(time, phi_list[1], 'g--',  label=r'$\phi_{des}(t)$')
theta.plot(time, theta_list[1], 'g--', label=r'$\theta_{des}(t)$')
psi.plot(time, psi_list[1], 'g--', label=r'$\psi_{des}(t)$')


phi.plot(time, phi_list[2], 'b',  label=r'$\phi_{est}(t)$')
theta.plot(time, theta_list[2], 'b', label=r'$\theta_{est}(t)$')
psi.plot(time, psi_list[2], 'b', label=r'$\psi_{est}(t)$')


# phi.plot(time, phi_list[3], 'y--', label=r'$\phi_{cam_1}(t)$', alpha=0.7)
# theta.plot(time, theta_list[3], 'y--', label=r'$\theta_{cam_1}(t)$', alpha=0.7)
# psi.plot(time, psi_list[3], 'y--', label=r'$\psi_{cam_1}(t)$', alpha=0.7)

# phi.plot(time, phi_list[4], 'k--', label=r'$\phi_{cam_2}(t)$', alpha=0.7)
# theta.plot(time, theta_list[4], 'k--', label=r'$\theta_{cam_2}(t)$', alpha=0.7)
# psi.plot(time, psi_list[4], 'k--', label=r'$\psi_{cam_2}(t)$', alpha=0.7)

psi.set_xlabel('Tempo(s)')

phi.set_ylabel(r'$\phi (rad)$')
theta.set_ylabel(r'$\theta (rad)$')
psi.set_ylabel(r'$\psi (rad)$')

phi.set_ylim(-0.1, 0.1)
theta.set_ylim(-0.1, 0.1)
psi.set_ylim(-0.1, 0.7)

phi.legend(bbox_to_anchor=(1.04,1), loc="upper left")
theta.legend(bbox_to_anchor=(1.04,1), loc="upper left")
psi.legend(bbox_to_anchor=(1.04,1), loc="upper left")

phi.grid()
theta.grid()
psi.grid()


#############################################################################################

#Plot gyro bias
fig_bias, bias_gyro = plt.subplots(1, 1, figsize=(15,10), sharex=True)
bias_gyro.plot(time, est_bias_g[0], 'r', label=r'$b_{g,x_{est}}$')
bias_gyro.plot(time, [0.2 for i in range(2000)], 'r--', label=r'$b_{g,x_{real}}$')
bias_gyro.plot(time, est_bias_g[1], 'g', label=r'$b_{g,y_{est}}$')
bias_gyro.plot(time, [0.4 for i in range(2000)], 'g--', label=r'$b_{g,y_{real}}$')
bias_gyro.plot(time, est_bias_g[2], 'b', label=r'$b_{g,z_{est}}$')
bias_gyro.plot(time, [-0.3 for i in range(2000)], 'b--', label=r'$b_{g,z_{real}}$')

bias_gyro.set_xlabel('Tempo(s)')

bias_gyro.set_ylabel(r'$b_g (rad/s)$')

bias_gyro.legend(bbox_to_anchor=(1.04,1), loc="upper left")
bias_gyro.legend(bbox_to_anchor=(1.04,1), loc="upper left")
bias_gyro.legend(bbox_to_anchor=(1.04,1), loc="upper left")

bias_gyro.grid()


#################################################################################################


fig_pos.savefig('results/Controle/position_' + case + '.png', bbox_inches='tight')
fig_quat.savefig('results/Controle/quaternion_' + case + '.png', bbox_inches='tight')
fig_euler.savefig('results/Controle/euler_' + case + '.png', bbox_inches='tight')
fig_bias.savefig('results/Controle/bias_' + case + '.png', bbox_inches='tight')

plt.show()


######################### ERROR ANALYSIS ####################################

#----------------- Estimation Analysis (Compare real simulation states with estimated EKF states) ------------------

#Mean squared error
# q0_mse = mean_squared_error(real_att[0], est_att[0])
# q1_mse = mean_squared_error(real_att[1], est_att[1])
# q2_mse = mean_squared_error(real_att[2], est_att[2])
# q3_mse = mean_squared_error(real_att[3], est_att[3])

# phi_mse = mean_squared_error(phi_list[0], phi_list[2])
# theta_mse = mean_squared_error(theta_list[0], theta_list[2])
# psi_mse = mean_squared_error(psi_list[0], psi_list[2])

# x_mse = mean_squared_error(real_pos[0], est_pos[0])
# y_mse = mean_squared_error(real_pos[1], est_pos[1])
# z_mse = mean_squared_error(real_pos[2], est_pos[2])


#----------------- Control Analysis (Compare desired states with estimated states) ------------------

# #Mean squared error
# q0_mse = mean_squared_error(des_att[0], est_att[0])
# q1_mse = mean_squared_error(des_att[1], est_att[1])
# q2_mse = mean_squared_error(des_att[2], est_att[2])
# q3_mse = mean_squared_error(des_att[3], est_att[3])

# phi_mse = mean_squared_error(phi_list[1], phi_list[2])
# theta_mse = mean_squared_error(theta_list[1], theta_list[2])
# psi_mse = mean_squared_error(psi_list[1], psi_list[2])

# x_mse = mean_squared_error(des_pos[0], est_pos[0])
# y_mse = mean_squared_error(des_pos[1], est_pos[1])
# z_mse = mean_squared_error(des_pos[2], est_pos[2])


# # # #RMSE
# q0_rmse = math.sqrt(q0_mse)
# q1_rmse = math.sqrt(q1_mse)
# q2_rmse = math.sqrt(q2_mse)
# q3_rmse = math.sqrt(q3_mse)

# phi_rmse = math.sqrt(phi_mse)
# theta_rmse = math.sqrt(theta_mse)
# psi_rmse = math.sqrt(psi_mse)

# x_rmse = math.sqrt(x_mse)
# y_rmse = math.sqrt(y_mse)
# z_rmse = math.sqrt(z_mse)

# print("ErEKF Quaternion RMSE: \n q0 - {0} \n q1 - {1} \n q2 - {2} \n q3 - {3}".format(q0_rmse, q1_rmse, q2_rmse, q3_rmse))
# print("ErEKF Euler RMSE (radianos): \n Roll - {0} \n Pitch - {1} \n Yaw - {2}".format(phi_rmse, theta_rmse, psi_rmse))
# print("Posição RMSE (metros): \n X - {0} \n Y - {1} \n Z - {2}".format(x_rmse, y_rmse, z_rmse))