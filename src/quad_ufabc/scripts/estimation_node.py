#!/usr/bin/env python3
import rospy
import numpy as np
import message_filters

from kalman_filter import KF
from collections import deque
from std_msgs.msg import Bool
from quad_ros import quad_robot
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32, Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import Quaternion, PoseStamped, Vector3Stamped, Pose, Vector3

from quat_utils import Quat2Rot

#Initialization variables
cam_att = None
cam2_att = None
cam_pos = None
cam2_pos = None

accel_raw = None
ang_vel = None

cam1_info = False
cam2_info = False


b_gx = np.random.normal(0.2, 1e-4)
b_gy = np.random.normal(0.4, 1e-4)
b_gz = np.random.normal(-0.3, 1e-4)

##################################### CALLBACKS ###########################################################

def callback_imu(data):

    """
    Recovers IMU data, where:
    accel_raw ---> linear acceleration in m/s^2 
    accel ---> normalized linear acceleration
    ang_vel ---> angular velocity in rad/s
    """

    global accel_raw, accel, ang_vel

    accel_raw = np.array([[data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]]).T

    ang_vel = np.array([[data.angular_velocity.x + b_gx, data.angular_velocity.y + b_gy, data.angular_velocity.z + b_gz]]).T

def callback_cv(pose, pose2, vec, vec2, info, info2):

    """
    Recovers data from computer vision node, where:
    cam_pos ---> Position vector provided by camera 1
    cam2_pos ---> Position vector provided by camera 2
    cam_att ---> Attitude vector (3D) provided by camera 1
    cam2_att ---> Attitude vector (3D) provided by camera 2
    cam1_info ---> Flag provided by camera 1 indicating that there is new data
    cam2_info ---> Flag provided by camera 2 indicating that there is new data
    """

    global cam_pos, cam2_pos, cam_att, cam2_att, cam1_info, cam2_info

    cam_pos = np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]]).T
    cam2_pos = np.array([[pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z]]).T
    cam_att = np.array([[vec.vector.x, vec.vector.y, vec.vector.z]]).T
    cam2_att = np.array([[vec2.vector.x, vec2.vector.y, vec2.vector.z]]).T
    cam1_info = info.data
    cam2_info = info2.data


###########################################################################################################

def noisy_cam_meas(curr_meas, meas_list, tolerance):

    """
    This function will identify if there is a noisy data provided by the camera, avoiding the ambiguity problem from camera's pose estimation.
    """

    #Check if the list has 10 vectors
    if len(meas_list)==10:
        
        #Compute median value from list
        meas_median = np.median(meas_list, axis=0)

        #Compute error beewteen the current data and the median
        error = abs(curr_meas - meas_median)
        #Verify if the error is greater than tolerance, if not, the current data is no noisy.
        is_no_noisy = (error < tolerance).any()

        #Check if the current data is noisy
        if is_no_noisy:
            
            #If is no noisy, such data will compose the list which contain the last 10 normal data
            meas_list.append(curr_meas)
        
        else:
            
            #If is noisy, the list will be clear
            meas_list.clear()

    else:

        #Include data in list
        meas_list.append(curr_meas)
        is_no_noisy = False
    
    return is_no_noisy


def EKF_node():

    """
    Function that publish all quadrotor's filtered positions and orientations.
    """

    #Initialize Estimator Node
    rospy.init_node('estimator')

    #Declare attitude publisher based on quaternion parametrization
    attitude_pub = rospy.Publisher('quad/kf/attitude', Quaternion, queue_size=10)
    position_pub = rospy.Publisher('quad/kf/position', Vector3, queue_size=10)
    bias_gyro_pub = rospy.Publisher('quad/kf/bias_gyro', Vector3, queue_size=10)
    trace_pub = rospy.Publisher('quad/kf/trace', Float32, queue_size=10)
    P_pub = rospy.Publisher('quad/kf/P_k', numpy_msg(Floats), queue_size=10)
    error_pub = rospy.Publisher('quad/kf/error_state', numpy_msg(Floats), queue_size=10)
    vel_pub = rospy.Publisher('quad/kf/vel', Vector3, queue_size=10)
    
    #Sensors subscribe
    imu_sub = rospy.Subscriber('/quad/imu', Imu, callback_imu, queue_size=10)

    #Define subscribers to obtain pose and camera vector from computer vision topics.
    cv_position_sub = message_filters.Subscriber('quad/computer_vision/pose', PoseStamped)
    cv2_position_sub = message_filters.Subscriber('quad/computer_vision/pose2', PoseStamped)
    cv_cam_vec_sub = message_filters.Subscriber('quad/computer_vision/cam_vec', Vector3Stamped)
    cv2_cam_vec_sub = message_filters.Subscriber('quad/computer_vision/cam2_vec', Vector3Stamped)
    cv_info_sub = message_filters.Subscriber('quad/computer_vision/info', Bool)
    cv2_info_sub = message_filters.Subscriber('quad/computer_vision/info2', Bool)

    #Apply message filter that synchronizer the messages from different topics
    ts = message_filters.ApproximateTimeSynchronizer([cv_position_sub, cv2_position_sub, cv_cam_vec_sub, cv2_cam_vec_sub, cv_info_sub, cv2_info_sub], queue_size=10, slop=0.5, allow_headerless=True)
    ts.registerCallback(callback_cv)


    #Instance KF class using sensors
    ekf = KF()
    #Define acquisition frequency
    rate = rospy.Rate(100)

    #Declare some useful variables

    #Variable that will store previous data
    cam_att_ant = None
    cam2_att_ant = None
    #List that save 10 values
    cam_meas_list = deque(maxlen=10)
    cam2_meas_list = deque(maxlen=10)
    #Flag that indicate if the current and previous measurement are equal
    cam1_equal = True
    cam2_equal = True
    #Flag that indicate if the new data is relevant to input in filter
    new_cam1 = False
    new_cam2 = False
    
    t_ant = 0
    
    dt = 0.01


    while not rospy.is_shutdown():

        #Instancce Quaternion class from geometry_msg which will store estimated quaternion from KF
        ang_est = Quaternion()
        pos_est = Vector3()
        vel_est = Vector3()
        ba_est = Vector3()
        bg_est = Vector3()
        trace = Float32()


        t = rospy.Time.now().to_nsec()
        dt = round((t - t_ant)*1e-9, 3)
        t_ant = t

        if dt > 0.02:

            dt = 0.01


        #Verify if it got some information from IMU
        if accel_raw is not None:

            #Verify if it got some information from Camera 1
            if cam_att is not None:
                
                #Check if the current and previous measurements are equal
                cam1_equal = np.array_equal(cam_att, cam_att_ant)
                #Check if the current measurement is noisy
                flag_cam1 = noisy_cam_meas(cam_att, cam_meas_list, 0.05)
                # flag_cam1 = False
                
                #Check if the current measurement is relevant 
                if not cam1_equal and flag_cam1 and cam1_info:
                    
                    new_cam1 = True

                else:

                    new_cam1 = False
                
            #Verify if it got some information from Camera 2
            if cam2_att is not None:
                
                #Check if the current and previous measurements are equal
                cam2_equal = np.array_equal(cam2_att, cam2_att_ant)
                #Check if the current measurement is noisy
                flag_cam2 = noisy_cam_meas(cam2_att, cam2_meas_list, 0.01)
                # flag_cam2 = False

                #Check if the current measurement is relevant 
                if not cam2_equal and flag_cam2 and cam2_info:

                    new_cam2 = True

                else:

                    new_cam2 = False


            #Verify if there is some relevant information from any camera
            if new_cam1 or new_cam2:
                
                # print('Camera')
                
                if new_cam1 and new_cam2:
    
                    #Apply Error State Kalman Filter using information from IMU, Camera 1 and Camera 2
                    ekf.ErEKF(accel_raw, ang_vel, cam_att, cam_pos, cam2_att, cam2_pos, dt)
                    # pass

                if new_cam1 and not new_cam2:
                    # print('Camera 1')    
                    #Apply Error State Kalman Filter using information from IMU and Camera 1
                    ekf.ErEKF(accel_raw, ang_vel, cam_att, cam_pos, None, None, dt)
                    # ekf.MEKF(accel_raw, ang_vel, cam_att)
                
                if new_cam2 and not new_cam1:
                    # print('Camera 2')
                    #Apply Error State Kalman Filter using information from IMU and Camera 2
                    ekf.ErEKF(accel_raw, ang_vel, None, None, cam2_att, cam2_pos, dt)
                    # ekf.MEKF(accel_raw, ang_vel, cam2_att)

            else:
                # print('IMU')
                #Apply Error State Kalman Filter using only IMU information
                ekf.ErEKF(accel_raw, ang_vel, None, None, None, None, dt)
                # ekf.MEKF(accel_raw, ang_vel, None)

            #Save the current measurement for next step
            cam_att_ant = cam_att
            cam2_att_ant = cam2_att
            
            #Store the estimated quaternion in message
            ang_est.x = ekf.q_K[1,0]
            ang_est.y = ekf.q_K[2,0]
            ang_est.z = ekf.q_K[3,0]
            ang_est.w = ekf.q_K[0,0]

            #Store the esimated position in message
            pos_est.x = ekf.pos_K[0,0]
            pos_est.y = ekf.pos_K[1,0]
            pos_est.z = ekf.pos_K[2,0]


            vel_est.x = ekf.v_K[0,0]
            vel_est.y = ekf.v_K[1,0]
            vel_est.z = ekf.v_K[2,0]

            #Store the esimated gyro bias in message
            bg_est.x = ekf.b_K[0,0]
            bg_est.y = ekf.b_K[1,0]
            bg_est.z = ekf.b_K[2,0]

            #Store Covariance Matrix

            #Store trace from P_K in EKF
            trace.data = float(ekf.trace)

            #Publish estimated quaternion on 'quad/kf/attitude' topic
            attitude_pub.publish(ang_est)

            #Publish estimated position on 'quad/kf/position' topic
            position_pub.publish(pos_est)

            vel_pub.publish(vel_est)

            #Publish estimated gyro bias on 'quad/kf/bias_gyro' topic
            bias_gyro_pub.publish(bg_est)

            #Publish EKF on 'quad/kf/trace' topic
            trace_pub.publish(trace)

            #Publish EKF on 'quad/kf/P_k' topic
            P_pub.publish(np.float32(ekf.P_K))

            #Publish EKF on 'quad/kf/error_state' topic
            error_pub.publish(np.float32(ekf.dx_k))

        rate.sleep()


if __name__ == '__main__':
    try:
        EKF_node()
    except rospy.ROSInterruptException:
        pass