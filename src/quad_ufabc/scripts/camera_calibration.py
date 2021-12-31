#!/usr/bin/env python3
import rospy
import pickle
import os, sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from quad_ros import quad_robot
from sklearn import linear_model
from geometry_msgs.msg import Pose, Vector3, PoseStamped, Vector3Stamped

#Set main directory
mydir = os.path.abspath(sys.path[0])


#Declare variables

#Positions
x_cam, y_cam, z_cam = 0, 0, 0
x_cam2, y_cam2, z_cam2 = 0, 0, 0

#Attitude vectors
x_cam_att, y_cam_att, z_cam_att = 0, 0, 0
x_cam2_att, y_cam2_att, z_cam2_att = 0, 0, 0

#IMU
accel_raw = None
accel = None
ang_vel = None

control_flag = False

def callback_positions(data):

    """
    Store positions from camera 1 pose topic
    """

    global x_cam, y_cam, z_cam

    x_cam = data.pose.position.x
    y_cam = data.pose.position.y
    z_cam = data.pose.position.z

def callback_positions2(data):

    """
    Store positions from camera 2 pose topic
    """

    global x_cam2, y_cam2, z_cam2

    x_cam2 = data.pose.position.x
    y_cam2 = data.pose.position.y
    z_cam2 = data.pose.position.z

def callback_control_flag(data):

    """
    Store flag from control flag topic
    """

    global control_flag

    control_flag = data

def callback_attitude_camera(data):
    
    """
    Store attitude vector from camera 1 topic
    """

    global x_cam_att, y_cam_att, z_cam_att

    x_cam_att = data.vector.x
    y_cam_att = data.vector.y
    z_cam_att = data.vector.z

def callback_attitude_camera2(data):

    """
    Store attitude vector from camera 2 topic
    """

    global x_cam2_att, y_cam2_att, z_cam2_att

    x_cam2_att = data.vector.x
    y_cam2_att = data.vector.y
    z_cam2_att = data.vector.z

def callback_imu(data):

    global accel_raw, accel, ang_vel

    accel_raw = np.array([[data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]]).T
    norm_accel = np.linalg.norm(accel_raw)
    accel = accel_raw/norm_accel

    ang_vel = np.array([[data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]]).T


def calib_position_camera():

    rospy.init_node('calibration')
    position_sub = rospy.Subscriber('quad/computer_vision/pose', PoseStamped, callback_positions, queue_size = 10)
    position_sub = rospy.Subscriber('quad/computer_vision/pose2', PoseStamped, callback_positions2, queue_size = 10)
    contro_flag_sub = rospy.Subscriber('quad/control/calib_flag', Bool, callback_control_flag, queue_size=10)

    quad_ufabc = quad_robot('quad')

    rate = rospy.Rate(10)

    x_cam_list, y_cam_list, z_cam_list = [], [], []
    x_cam2_list, y_cam2_list, z_cam2_list = [], [], []
    x_real_list, y_real_list, z_real_list = [], [], []

    while not rospy.is_shutdown():
        
        if control_flag:

            data_dict = {'x meas1': x_cam_list, 'y meas1': y_cam_list, 'z meas1': z_cam_list,
                        'x meas2': x_cam2_list, 'y meas2': y_cam2_list, 'z meas2': z_cam2_list,
                    'x real': x_real_list, 'y real': y_real_list, 'z real': z_real_list}
                    
            df = pd.DataFrame.from_dict(data_dict)

            x_values = df[['x meas1', 'x real']]
            y_values = df[['y meas1', 'y real']]
            z_values = df[['z meas1', 'z real']]

            x2_values = df[['x meas2', 'x real']]
            y2_values = df[['y meas2', 'y real']]
            z2_values = df[['z meas2', 'z real']]

            x_mat = np.array(x_values, 'float')
            y_mat = np.array(y_values, 'float')
            z_mat = np.array(z_values, 'float')

            x2_mat = np.array(x2_values, 'float')
            y2_mat = np.array(y2_values, 'float')
            z2_mat = np.array(z2_values, 'float')

            x_input = x_mat[:, 0].reshape(-1, 1)
            x2_input = x2_mat[:, 0].reshape(-1, 1)
            x_target = x_mat[:, 1].reshape(-1, 1)

            y_input = y_mat[:, 0].reshape(-1, 1)
            y2_input = y2_mat[:, 0].reshape(-1, 1)
            y_target = y_mat[:, 1].reshape(-1, 1)

            z_input = z_mat[:, 0].reshape(-1, 1)
            z2_input = z2_mat[:, 0].reshape(-1, 1)
            z_target = z_mat[:, 1].reshape(-1, 1)

            print(np.shape(x_input), np.shape(x_target))

            rl_x = linear_model.LinearRegression()
            rl_x.fit(x_input, x_target)

            rl_y = linear_model.LinearRegression()
            rl_y.fit(y_input, y_target)

            rl_z = linear_model.LinearRegression()
            rl_z.fit(z_input, z_target)

            rl_x2 = linear_model.LinearRegression()
            rl_x2.fit(x2_input, x_target)

            rl_y2 = linear_model.LinearRegression()
            rl_y2.fit(y2_input, y_target)

            rl_z2 = linear_model.LinearRegression()
            rl_z2.fit(z2_input, z_target)

            print('Score X', rl_x.score(x_input, x_target))
            print('Score Y', rl_y.score(y_input, y_target))
            print('Score Z', rl_z.score(z_input, z_target))

            print('Score X2', rl_x2.score(x2_input, x_target))
            print('Score Y2', rl_y2.score(y2_input, y_target))
            print('Score Z2', rl_z2.score(z2_input, z_target))


            print('Coeff X:', [rl_x.coef_, rl_x.intercept_])
            print('Coeff Y:', [rl_y.coef_, rl_y.intercept_])
            print('Coeff Z:', [rl_z.coef_, rl_z.intercept_])

            calib_dict = {'theta x': [rl_x.coef_, rl_x.intercept_], 'theta y':  [rl_y.coef_, rl_y.intercept_], 'theta z': [rl_z.coef_, rl_z.intercept_],
                            'theta x2': [rl_x2.coef_, rl_x2.intercept_], 'theta y2':  [rl_y2.coef_, rl_y2.intercept_], 'theta z2': [rl_z2.coef_, rl_z2.intercept_],
                            'x input': x_mat[:, 0], 'y input': y_mat[:, 0], 'z input': z_mat[:, 0],
                            'x2 input': x2_mat[:, 0], 'y2 input': y2_mat[:, 0], 'z2 input': z2_mat[:, 0],
                            'x real:': x_mat[:, 1], 'y real': y_mat[:, 1], 'z real': z_mat[:, 1]}

            outfile = open(mydir+'/'+'data/data_calibration.p', 'wb')

            pickle.dump(calib_dict, outfile)
            outfile.close()

        else:
            
            print('Collecting Data...')

            x_cam_list.append(x_cam)
            y_cam_list.append(y_cam)
            z_cam_list.append(z_cam)

            x_cam2_list.append(x_cam2)
            y_cam2_list.append(y_cam2)
            z_cam2_list.append(z_cam2)

            x_real_list.append(quad_ufabc.position[0])
            y_real_list.append(quad_ufabc.position[1])
            z_real_list.append(quad_ufabc.position[2])


        rate.sleep()
        

def compute_std():

    rospy.init_node('standard_deviation')

    #Subscribe in topics published by camera 1
    cam_pose_sub = rospy.Subscriber('quad/computer_vision/pose', PoseStamped, callback_positions, queue_size = 10)
    cam_att_sub = rospy.Subscriber('quad/computer_vision/cam_vec', Vector3Stamped, callback_attitude_camera, queue_size = 10)

    #Subscribe in topics published by camera 2
    cam2_pose_sub = rospy.Subscriber('quad/computer_vision/pose2', PoseStamped, callback_positions2, queue_size = 10)
    cam2_att_sub = rospy.Subscriber('quad/computer_vision/cam2_vec', Vector3Stamped, callback_attitude_camera2, queue_size = 10)

    #Sensors subscribe
    imu_sub = rospy.Subscriber('/quad/imu', Imu, callback_imu, queue_size=10)

    #Subscribe in topic published by controller when the trajectory comes to an end
    contro_flag_sub = rospy.Subscriber('quad/control/calib_flag', Bool, callback_control_flag, queue_size=10)

    #Set data rate
    rate = rospy.Rate(10)
    
    #Declare some empty lists
    x_cam_list, y_cam_list, z_cam_list = [], [], []
    x_att_cam_list, y_att_cam_list, z_att_cam_list = [], [], []

    x_cam2_list, y_cam2_list, z_cam2_list = [], [], []
    x_att_cam2_list, y_att_cam2_list, z_att_cam2_list = [], [], []

    x_accel_list, y_accel_list, z_accel_list = [], [], []
    x_accel_n_list, y_accel_n_list, z_accel_n_list = [], [], []
    x_gyro_list, y_gyro_list, z_gyro_list = [], [], []

    #Initialize count variable
    count = 0

    while not rospy.is_shutdown():

        
        if len(x_cam_list) < 5000 and accel_raw is not None:
            
            count += 1

            print('Collecting data from sensors -->', count)

            # Camera 1 - store data
            x_cam_list.append(x_cam)
            y_cam_list.append(y_cam)
            z_cam_list.append(z_cam)

            x_att_cam_list.append(x_cam_att)
            y_att_cam_list.append(y_cam_att)
            z_att_cam_list.append(z_cam_att)

            # Camera 2 - store data
            x_cam2_list.append(x_cam2)
            y_cam2_list.append(y_cam2)
            z_cam2_list.append(z_cam2)

            x_att_cam2_list.append(x_cam2_att)
            y_att_cam2_list.append(y_cam2_att)
            z_att_cam2_list.append(z_cam2_att)

            # Accelerometer - store data
            x_accel_list.append(accel_raw[0,0])
            y_accel_list.append(accel_raw[1,0])
            z_accel_list.append(accel_raw[2,0])

            #Accelerometer normalized - store data
            x_accel_n_list.append(accel[0,0])
            y_accel_n_list.append(accel[1,0])
            z_accel_n_list.append(accel[2,0])

            #Gyro - store data
            x_gyro_list.append(ang_vel[0,0])
            y_gyro_list.append(ang_vel[1,0])
            z_gyro_list.append(ang_vel[2,0])
        
        elif len(x_cam_list) == 5000:
            
            # Camera 1 Standard Deviation
            std_x_cam = np.std(x_cam_list)
            std_y_cam = np.std(y_cam_list)
            std_z_cam = np.std(z_cam_list)

            std_x_cam_att = np.std(x_att_cam_list)
            std_y_cam_att = np.std(y_att_cam_list)
            std_z_cam_att = np.std(z_att_cam_list)
            
            # Camera 2 Standard Deviation
            std_x_cam2 = np.std(x_cam2_list)
            std_y_cam2 = np.std(y_cam2_list)
            std_z_cam2 = np.std(z_cam2_list)

            std_x_cam2_att = np.std(x_att_cam2_list)
            std_y_cam2_att = np.std(y_att_cam2_list)
            std_z_cam2_att = np.std(z_att_cam2_list)

            #Accelerometer Standard Deviation
            std_x_accel = np.std(x_accel_list)
            std_y_accel = np.std(y_accel_list)
            std_z_accel = np.std(z_accel_list)

            #Accelerometer Normalized Standard Deviation
            std_x_accel_n = np.std(x_accel_n_list)
            std_y_accel_n = np.std(y_accel_n_list)
            std_z_accel_n = np.std(z_accel_n_list)

            #Gyro Standard Deviation
            std_x_gyro = np.std(x_gyro_list)
            std_y_gyro = np.std(y_gyro_list)
            std_z_gyro = np.std(z_gyro_list)

            #Store standard deviation in a dictionary
            std_dict = {'accel': np.array([[std_x_accel, std_y_accel, std_z_accel]]).T,
                        'accel norm': np.array([[std_x_accel_n, std_y_accel_n, std_z_accel_n]]).T, 
                        'gyro': np.array([[std_x_gyro, std_y_gyro, std_z_gyro]]).T, 
                        'cam pos': np.array([[std_x_cam, std_y_cam, std_z_cam]]).T, 
                        'cam att': np.array([[std_x_cam_att, std_y_cam_att, std_z_cam_att]]).T,
                        'cam2 pos': np.array([[std_x_cam2, std_y_cam2, std_z_cam2]]).T, 
                        'cam2 att': np.array([[std_x_cam2_att, std_y_cam2_att, std_z_cam2_att]]).T}

            #Save dictionary in a pickle file
            outfile = open(mydir+'/'+'data/std_sensors.p', 'wb')
            pickle.dump(std_dict, outfile)
            outfile.close()

            print('Standard Deviations computed!')
            
        rate.sleep()




if __name__ == '__main__':
    try:
        calib_position_camera()
        # compute_std()
    except rospy.ROSInterruptException:
        pass