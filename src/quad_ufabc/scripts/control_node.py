#!/usr/bin/env python3
import rospy
import pickle
import os, sys
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from quad_ros import quad_robot
from quat_utils import QuatProd
from quad_control import Controller
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Vector3, Quaternion, Pose, Vector3Stamped, PoseStamped

from sensor_msgs.msg import Imu

#Set main directory
mydir = os.path.abspath(sys.path[0])



################################################################################################################################

#Initial estimated states

#Quaternion estimated by EKF
qw_est, qx_est, qy_est, qz_est = 1, 0, 0, 0

#Position estimated by EKF
x_est, y_est, z_est = 0.6, 0, 0

#Position measured by camera 1
x_cam, y_cam, z_cam = 0.6, 0, 0

#Position measured by camera 2
x_cam2, y_cam2, z_cam2 = 0.6, 0, 0

#QUaternion measured by camera 1
q0_cam, q1_cam, q2_cam, q3_cam = 1, 0, 0, 0

#QUaternion measured by camera 2
q0_cam2, q1_cam2, q2_cam2, q3_cam2 = 1, 0, 0, 0

#Velocity
vx, vy, vz = 0, 0, 0

#Trace from EKF
trace = np.trace(np.diag([1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3]))

#Accelerometer
ax, ay, az = 0, 0, 0

#Gyro
gx, gy, gz = 0, 0, 0

#Bias Gyro
bg_x, bg_y, bg_z = 0, 0, 0

#Initial covariance matrix
P = np.array([1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3], dtype='float32')

#Initial error state
error_state = np.zeros([12,1])
##################################### CALLBACKS ###########################################################

def callback_IMU(data):
    
    """
    Recovers IMU data, where:
    ax ---> linear acceleration in X axis (m/s^2)
    ay ---> linear acceleration in Y axis (m/s^2)
    az ---> linear acceleration in Z axis (m/s^2)

    gx ---> angular velocity in X axis (rad/s)
    gy ---> angular velocity in Y axis (rad/s)
    gz ---> angular velocity in Z axis (rad/s)
    """

    global ax, ay, az, gx, gy, gz, q0_imu, q1_imu, q2_imu, q3_imu

    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y
    az = data.linear_acceleration.z

    gx = data.angular_velocity.x
    gy = data.angular_velocity.y
    gz = data.angular_velocity.z

 

def callback_cam_pose(data):

    """
    Recovers pose estimated by camera 1, where:

    x_cam ---> X quadrotor's position in inertial frame (m)
    y_cam ---> Y quadrotor's position in inertial frame (m)
    z_cam ---> Z quadrotor's position in inertial frame (m)

    q0_cam ---> scalar component from camera 1's quaternion estimation (Hamilton convention)
    q1_cam ---> x vector component from camera 1's quaternion estimation (Hamilton convention)
    q2_cam ---> y vector component from camera 1's quaternion estimation (Hamilton convention)
    q3_cam ---> z vector component from camera 1's quaternion estimation (Hamilton convention)

    """

    global x_cam, y_cam, z_cam, q0_cam, q1_cam, q2_cam, q3_cam

    x_cam = data.pose.position.x
    y_cam = data.pose.position.y
    z_cam = data.pose.position.z

    q0_cam = data.pose.orientation.w
    q1_cam = data.pose.orientation.x
    q2_cam = data.pose.orientation.y
    q3_cam = data.pose.orientation.z

def callback_cam2_pos(data):

    """
    Recovers pose estimated by camera 2, where:

    x_cam2 ---> X quadrotor's position in inertial frame (m)
    y_cam2 ---> Y quadrotor's position in inertial frame (m)
    z_cam2 ---> Z quadrotor's position in inertial frame (m)

    q0_cam2 ---> scalar component from camera 2's quaternion estimation (Hamilton convention)
    q1_cam2 ---> x vector component from camera 2's quaternion estimation (Hamilton convention)
    q2_cam2 ---> y vector component from camera 2's quaternion estimation (Hamilton convention)
    q3_cam2 ---> z vector component from camera 2's quaternion estimation (Hamilton convention)
    
    """

    global x_cam2, y_cam2, z_cam2, q0_cam2, q1_cam2, q2_cam2, q3_cam2

    x_cam2 = data.pose.position.x
    y_cam2 = data.pose.position.y
    z_cam2 = data.pose.position.z

    q0_cam2 = data.pose.orientation.w
    q1_cam2 = data.pose.orientation.x
    q2_cam2 = data.pose.orientation.y
    q3_cam2 = data.pose.orientation.z

def callback_attitude(data):

    """
    Recovers quaternion estimated by ErEKF algorithm
    """

    global qx_est, qy_est, qz_est, qw_est

    qx_est = data.x
    qy_est = data.y
    qz_est = data.z
    qw_est = data.w

def callback_position(data):

    """
    Recovers position estimated by ErEKF algorithm
    """

    global x_est, y_est, z_est

    x_est = data.x
    y_est = data.y
    z_est = data.z

def callback_trace(data):

    """
    Recovers trace from ErEKF
    """

    global trace

    trace = data.data

def callback_bias_gyro(data):

    """
    Recovers estimated gyro bias from ErEKF
    """

    global bg_x, bg_y, bg_z

    bg_x = data.x
    bg_y = data.y
    bg_z = data.z

def callback_p(data):

    global P

    P = data.data

def callback_error(data):

    global error_state

    error_state = data.data

def callback_velocity(data):

    global vx, vy, vz

    vx = data.x
    vy = data.y
    vz = data.z


#############################################################################################################

def controller_node():

    """
    Function that defines the controller node. The controller is responsible for the attitude and position tracking of the quadrotor.
    """

    #Initialize controller node
    rospy.init_node('controller', anonymous=True)
    #Define the subscribers which will get the estimated quaternions and positions from ErEKF
    attitude_sub = rospy.Subscriber('quad/kf/attitude', Quaternion, callback_attitude, queue_size=10)
    position_sub = rospy.Subscriber('quad/kf/position', Vector3, callback_position, queue_size = 10)
    trace_sub = rospy.Subscriber('quad/kf/trace', Float32, callback_trace, queue_size=10)
    bias_gyro_sub = rospy.Subscriber('quad/kf/bias_gyro', Vector3, callback_bias_gyro, queue_size=10)
    vel_sub = rospy.Subscriber('quad/kf/vel', Vector3, callback_velocity, queue_size=10)

    #Define subscribers which will get the measurements from cameras
    cam_pos_sub = rospy.Subscriber('quad/computer_vision/pose', PoseStamped, callback_cam_pose, queue_size=10)
    cam2_pos_sub = rospy.Subscriber('quad/computer_vision/pose2', PoseStamped, callback_cam2_pos, queue_size=10)

    #Define subscribers IMU
    IMU_sub = rospy.Subscriber('quad/imu', Imu, callback_IMU, queue_size=10)

    #If the position measured by the camera need some calibration, the controller will publish when the desired trajectory is finished
    flag_calib_pub = rospy.Publisher('quad/control/calib_flag', Bool, queue_size=10)

    #Quadrotor and Control instances
    quad_ufabc = quad_robot('quad')
    controller = Controller()

    quad_ufabc.reset()
    quad_ufabc.step([0, 0, 0, 0])


    ############################# TRAJECTORY PLANNER ###############################################

    #Import the desired trajectory computed by 'trajectory_generator.py' executable.
    infile = open(mydir + '/'+'data/trajectory_3.p', 'rb')
    traj = pickle.load(infile)
    infile.close()

    #Reset step
    step = 0

    ################################################################################################
    
    #Declare some list for plotting
    x_list, x_real_list, y_list, y_real_list, z_list, z_real_list = [], [], [], [], [], []
    x_est_list, y_est_list, z_est_list = [], [], []
    x_cam_list, y_cam_list, z_cam_list = [], [], []
    x_cam2_list, y_cam2_list, z_cam2_list = [], [], []

    vx_list, vy_list, vz_list = [], [], []
    vx_real_list, vy_real_list, vz_real_list = [], [], []

    q0_list, q1_list, q2_list, q3_list = [], [], [], []
    q0_cam_list, q1_cam_list, q2_cam_list, q3_cam_list = [], [], [], []
    q0_cam2_list, q1_cam2_list, q2_cam2_list, q3_cam2_list = [], [], [], []
    q0_est_list, q1_est_list, q2_est_list, q3_est_list = [], [], [], []
    q0_des_list, q1_des_list, q2_des_list, q3_des_list = [], [], [], []

    ax_list, ay_list, az_list = [], [], []
    gx_list, gy_list, gz_list = [], [], []

    ag_x_list, ag_y_list, ag_z_list = [], [], []
    bg_x_list, bg_y_list, bg_z_list = [], [], []
    P_list = []
    error_list = []

    trace_list = []
    
    k = 0
    #Flag to plot
    flag_plot =  True

    while not rospy.is_shutdown():
        
        #Instance flag to calibration process 
        flag_calib = Bool()
        position_real = Pose()
        
        #While step is smaller than the trajectory size
        if step < len(traj['x']):

            #Translation real states
            pos_real = np.asarray(quad_ufabc.position).reshape(3,1)
            vel_real = np.asarray(quad_ufabc.velocity).reshape(3,1)

            #Set desired states from generated trajectory for each step
            pos_ref = np.array([[traj['x'][step], traj['y'][step], traj['z'][step]]]).T
            vel_ref = np.array([[traj['dx'][step], traj['dy'][step], traj['dz'][step]]]).T
            accel_ref = np.array([[traj['ddx'][step], traj['ddy'][step], traj['ddz'][step]]]).T
            qz = traj['qz ref'][step]

            
            #Estimated states from Error State Kalman Filter
            q_est = np.array([[qw_est, qx_est, qy_est, qz_est]]).T

            pos_est = np.array([[x_est, y_est, z_est]]).T
            
            #Angular rate from gyroscope
            omega_gyro = np.array([[gx, gy, gz]]).T

            #Position Control based in quaternion parametrization
            T, q_pdes = controller.pos_control_quat(pos_est, pos_ref, vel_real, vel_ref, accel_ref)

            #Compute the desired quaternion
            q_des = QuatProd(q_pdes, qz)

            #Get real quaternions
            q = np.asarray(quad_ufabc.attitude_quat).reshape(4,1)
            
            
            # #Get real angular velocities 
            omega = np.asarray(quad_ufabc.angular_vel).reshape(3,1)
            

            #Attitude Control based in quaternion parametrization
            tau, error = controller.att_control_quat(q_est, q_des, omega_gyro)

            #Compute the rotors speeds from obtained inputs
            w, _, _ = controller.f2w(T, [tau[0,0], tau[1,0], -tau[2,0]])    

            w[0] = -w[0]
            w[2] = -w[2]


            #Send the command to quadrotor
            quad_ufabc.step(w/10)


            ##################   Append lists  ####################################################

            #Store real position in lists
            x_real_list.append(pos_real[0,0])
            y_real_list.append(pos_real[1,0])
            z_real_list.append(pos_real[2,0])
            
            #Store real velocity in lists
            vx_real_list.append(vel_real[0, 0])
            vy_real_list.append(vel_real[1, 0])
            vz_real_list.append(vel_real[2, 0])

            #Store real quaternion in lists
            q0_list.append(q[0,0])
            q1_list.append(q[1,0])
            q2_list.append(q[2,0])
            q3_list.append(q[3,0])

            #Store desired position in lists
            x_list.append(pos_ref[0,0])
            y_list.append(pos_ref[1,0])
            z_list.append(pos_ref[2,0])

            #Store desired quaternion in lists
            q0_des_list.append(q_des[0,0])
            q1_des_list.append(q_des[1,0])
            q2_des_list.append(q_des[2,0])
            q3_des_list.append(q_des[3,0])

            #Store estimated states from ErEKF
            x_est_list.append(x_est)
            y_est_list.append(y_est)
            z_est_list.append(z_est)

            vx_list.append(vx)
            vy_list.append(vy)
            vz_list.append(vz)

            q0_est_list.append(qw_est)
            q1_est_list.append(qx_est)
            q2_est_list.append(qy_est)
            q3_est_list.append(qz_est)

            bg_x_list.append(bg_x)
            bg_y_list.append(bg_y)
            bg_z_list.append(bg_z)

            #Store covariance matrix from ErKF
            P_list.append(P)

            #Store trace from covariance matrix
            trace_list.append(trace)

            #Store inovation from ErKF
            error_list.append(error_state)

            #Store IMU measurements

            ax_list.append(ax)
            ay_list.append(ay)
            az_list.append(az)

            gx_list.append(gx)
            gy_list.append(gy)
            gz_list.append(gz)


            #Store position camera measurements
            x_cam_list.append(x_cam)
            y_cam_list.append(y_cam)
            z_cam_list.append(z_cam)

            x_cam2_list.append(x_cam2)
            y_cam2_list.append(y_cam2)
            z_cam2_list.append(z_cam2)

            #Store quaternion camera

            q0_cam_list.append(q0_cam)
            q1_cam_list.append(q1_cam)
            q2_cam_list.append(q2_cam)
            q3_cam_list.append(q3_cam)

            q0_cam2_list.append(q0_cam2)
            q1_cam2_list.append(q1_cam2)
            q2_cam2_list.append(q2_cam2)
            q3_cam2_list.append(q3_cam2)

        
        else:
            
            #Indicates that the trajectory is finished
            flag_calib.data = True
            flag_calib_pub.publish(flag_calib)
            
            #Turn off quadrotor
            quad_ufabc.step([0, 0, 0, 0])

            #Create dictionary containing all positions and attitudes
            states_dict = {'real position': [x_real_list, y_real_list, z_real_list],
                           'desired position': [x_list, y_list, z_list], 
                           'measured position camera 1': [x_cam_list, y_cam_list, z_cam_list],
                           'measured position camera 2': [x_cam2_list, y_cam2_list, z_cam2_list],
                           'estimated position': [x_est_list, y_est_list, z_est_list],
                           'real attitude': [q0_list, q1_list, q2_list, q3_list],
                           'measured attitude camera 1': [q0_cam_list, q1_cam_list, q2_cam_list, q3_cam_list],
                           'measured attitude camera 2': [q0_cam2_list, q1_cam2_list, q2_cam2_list, q3_cam2_list],
                           'desired attitude': [q0_des_list, q1_des_list, q2_des_list, q3_des_list],
                           'estimated attitude': [q0_est_list, q1_est_list, q2_est_list, q3_est_list],
                           'trace': trace_list, 
                           'estimated gyro bias': [bg_x_list, bg_y_list, bg_z_list],
                           'accel meas': [ax_list, ay_list, az_list],
                           'gyro meas': [gx_list, gy_list, gz_list],
                           'P': P_list,
                           'error state': error_list,
                           'vel': [vx_list, vy_list, vz_list],
                           'vel_real': [vx_real_list, vy_real_list, vz_real_list]}
            
            outfile = open(mydir + '/' + 'data/states.p', 'wb')
            pickle.dump(states_dict, outfile)
            outfile.close()


            plt.show()
            
            
        #Increment step

        step += 1


        quad_ufabc.rate.sleep()


if __name__ == '__main__':
    try:
        controller_node()
    except rospy.ROSInterruptException:
        pass