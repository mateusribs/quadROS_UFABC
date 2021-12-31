#!/usr/bin/env python3
import rospy
import pickle
import os, sys
import cv2 as cv
import numpy as np
import scipy as sci

from quad_ros import quad_robot
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from scipy.spatial.transform import Rotation as rot
from geometry_msgs.msg import PoseStamped, Vector3Stamped

#Set main directory
mydir = os.path.abspath(sys.path[0])

#Check if position calibration data exists
if os.path.exists(mydir + '/' + 'data/data_calibration.p'):
            
    infile = open(mydir + '/' + 'data/data_calibration.p', 'rb')
    data = pickle.load(infile)
    infile.close()


image = None
image2 = None
mtx, dist = None, None
mtx2, dist2 = None, None

##################################### CALLBACKS ###########################################################

def callback_image(data):
    """
    Recovers front image from the camera mounted on the robot
    """

    global image

    image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

def callback_camerainfo(data):

    global mtx, dist

    K = data.K
    D = data.D

    mtx = np.array([[K[0], K[1], K[2]], [K[3], K[4], K[5]], [K[6], K[7], K[8]]])
    dist = np.array([[D[0], D[1], D[2], D[3], D[4]]])

def callback_image2(data):
    """
    Recovers front image from the camera mounted on the robot
    """

    global image2

    image2 = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

def callback_camerainfo2(data):

    global mtx2, dist2

    K = data.K
    D = data.D

    mtx2 = np.array([[K[0], K[1], K[2]], [K[3], K[4], K[5]], [K[6], K[7], K[8]]])
    dist2 = np.array([[D[0], D[1], D[2], D[3], D[4]]])

##########################################################################################################

def aruco_detection(image, dictionary, parameters, rvecs, tvecs, mtx, dist):

        """
        This function performs aruco's detection process, estimates the pose estimation from each marker w.r.t camera's frame and give us the homogeneous transformation that relates the fixed marker's frame to moving marker's frame.

        Input:
        image => acquired image
        dictionary => ArUco dictionary selected
        rvec => camera rotation vector -> camera calibration parameter
        tvec => camera translation vector -> camera calibration parameter
        mtx => camera's intrinsic matrix -> camera calibration parameter
        dist => camera's distortion vector -> camera calibration parameter

        Outputs: 
        position => relative position beetween fixed marker to moving marker;
        q_obj_b => quaternion that gives the transformation from inertial frame to body frame;
        cam_vec => attitude vector obtained from q_obj_b rotation matrix (1st column) which indicates the X-axis direction of body frame w.r.t inertial frame;
        image => processed image.
        """

        position = None
        q_obj_b = None
        q_obj = None
        cam_vec = None
        camera_meas_flag = None
        # n_cam = None

        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)


        #Detect the markers in the image
        markerCorners, markerIDs, rejectedCandidates = cv.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        
        

        #If there is a marker compute the pose
        if markerIDs is not None and len(markerIDs)==2:

            #Descending order Marker ID's array
            if len(markerIDs)==2:
                order = np.argsort(-markerIDs.reshape(1,2))
                markerIDs = markerIDs[order].reshape(2,1)
                markerCorners = np.asarray(markerCorners, dtype='float32')[order]

            for i in range(0, len(markerIDs)):
                
                #Check if it's reference marker
                if markerIDs[i]==10:
                    
                    #Compute Pose Estimation of the Reference Marker
                    rvec_ref, tvec_ref, _ = cv.aruco.estimatePoseSingleMarkers(markerCorners[0][i], 0.3, mtx, dist, rvecs, tvecs)
                    rvec_ref = np.reshape(rvec_ref, (3,1))
                    tvec_ref = np.reshape(tvec_ref, (3,1))
                    #Use Rodrigues formula to transform rotation vector into matrix
                    #Pose marker w.r.t camera reference frame
                    R_rc, _ = cv.Rodrigues(rvec_ref)
                    #Homogeneous Transformation Fixed Frame to Camera Frame
                    last_col = np.array([[0, 0, 0, 1]])
                    T_rc = np.concatenate((R_rc, tvec_ref), axis=1)
                    T_rc = np.concatenate((T_rc, last_col), axis=0)
                    #Homegeneous Transformation Camera Frame to Fixed Frame
                    T_cr = np.linalg.inv(T_rc)

                    #Get Reference's Marker attitude w.r.t camera
                    r_ref = sci.spatial.transform.Rotation.from_matrix(T_cr[0:3, 0:3])
                    q_ref = r_ref.as_quat()
                    euler_ref = r_ref.as_euler('XYZ')

                    #Draw axis in marker
                    cv.aruco.drawAxis(image, mtx, dist, rvec_ref, tvec_ref, 0.2)
                    cv.aruco.drawDetectedMarkers(image, markerCorners[0])

                #Check if there is moving marker/object marker
                if markerIDs[i]==4:
                

                    #Get Pose Estimation of the Moving Marker
                    rvec_obj, tvec_obj, _ = cv.aruco.estimatePoseSingleMarkers(markerCorners[0][i], 0.15, mtx, dist, rvecs, tvecs)
                    rvec_obj = np.reshape(rvec_obj, (3,1))
                    tvec_obj = np.reshape(tvec_obj, (3,1))

                    #Use Rodrigues formula to transform rotation vector into matrix
                    R_mc, _ = cv.Rodrigues(rvec_obj)

                    #Homogeneous Transformation Object Frame to Camera Frame
                    last_col = np.array([[0, 0, 0, 1]])
                    T_mc = np.concatenate((R_mc, tvec_obj), axis=1)
                    T_mc = np.concatenate((T_mc, last_col), axis=0)

                    # r_mov = sci.spatial.transform.Rotation.from_matrix(T_mc[0:3, 0:3])
                    # q_mov = r_mov.as_quat()
                    # euler_mov = r_mov.as_euler('XYZ')
                    

                    if T_cr is not None:
                        #Homogeneous Transformation Object Frame to Fixed Frame
                        T_mr = T_cr@T_mc
                    else:
                        T_mr = np.eye(4)@T_mc

                    T_rm = T_mr.T

                    #Getting quaternions from rotation matrix
                    r_obj = sci.spatial.transform.Rotation.from_matrix(T_mr[0:3, 0:3])
                    q_obj = r_obj.as_quat()
                    # euler_obj = r_obj.as_euler('XYZ')
                    
                    #Getting quaternion from Fixed Frame to Body Frame
                    r_obj_b = sci.spatial.transform.Rotation.from_matrix(T_rm[0:3, 0:3])
                    q_obj_b = r_obj_b.as_quat()


                    #Marker's Position
                    zf_obj = float(T_mr[2,3])
                    xf_obj = float(T_mr[0,3])
                    yf_obj = float(T_mr[1,3])

                    position = np.array([[xf_obj, yf_obj, zf_obj]]).T


                    #Measurement Direction Vector
                        
                    cam_vec = T_rm[0:3, 0:3]@np.array([[1, 0, 0]]).T
                                     

                    #Draw ArUco contourn and Axis
                    cv.aruco.drawAxis(image, mtx, dist, rvec_obj, tvec_obj, 0.15)
                    cv.aruco.drawDetectedMarkers(image, markerCorners[0])

        return position, q_obj, cam_vec, image



def computer_vision_publisher():
    
    """
    Function that publish all quadrotor's positions and orientations provided by camera sensor.
    """
    
    #Declare subscribe to recover camera plug in data
    camera_sub = rospy.Subscriber('/camera/camera1/image_raw', Image, callback_image, queue_size=10)
    camera_info_sub = rospy.Subscriber('/camera/camera1/camera_info', CameraInfo, callback_camerainfo, queue_size=10)
    camera2_sub = rospy.Subscriber('/camera2/camera2/image_raw_2', Image, callback_image2, queue_size=10)
    camera2_info_sub = rospy.Subscriber('/camera2/camera2/camera_info_2', CameraInfo, callback_camerainfo2, queue_size=10)

    #Declare pose obtained by camera 1 publisher
    pose_pub = rospy.Publisher('quad/computer_vision/pose', PoseStamped, queue_size=10)
    #Declare reference attitude camera 1 vector publisher
    cam_vec_pub = rospy.Publisher('quad/computer_vision/cam_vec', Vector3Stamped, queue_size=10)

    #Declare pose obtained by camera 2 publisher
    pose2_pub = rospy.Publisher('quad/computer_vision/pose2', PoseStamped, queue_size=10)
    #Declare reference attitude camera 2 vector publisher
    cam2_vec_pub = rospy.Publisher('quad/computer_vision/cam2_vec', Vector3Stamped, queue_size=10)
    #Declare flags from cameras
    cam1_info_pub = rospy.Publisher('quad/computer_vision/info', Bool, queue_size=10)
    cam2_info_pub = rospy.Publisher('quad/computer_vision/info2', Bool, queue_size=10)

    #Initialize Computer Vision node
    rospy.init_node('computer_vision')

    #Quadrotor Class ROS - Reset the states
    quad_ufabc = quad_robot('quad')
    quad_ufabc.reset()
    quad_ufabc.step([0, 0, 0, 0])

    #Set data rate
    rate = rospy.Rate(15)

    #Instance Pose class from geometry_msg which will store quadrotos's pose provided by cameras
    pose = PoseStamped()
    pose2 = PoseStamped()
    #Instance Vector3 class from geometry_msg which will store attitude cameras vectors.
    cam_vec = Vector3Stamped()
    cam2_vec = Vector3Stamped()
    #Instace Bool class from std_msgs which will store flags from cameras
    flag_cam1 = Bool()
    flag_cam2 = Bool()

    while not rospy.is_shutdown():

        rvecs = None
        tvecs = None
        
        global T_cr

        #Font setup
        font = cv.FONT_HERSHEY_PLAIN
        
        #Load the predefinied dictionary
        dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)

        
        ####################################### ARUCO MARKER POSE ESTIMATION #########################################################################


        #Verify if we got some information from camera 1 sensor
        if mtx is not None and dist is not None:
            

            #Initialize the detector parameters using defaults values
            cam1_param = cv.aruco.DetectorParameters_create()
            cam1_param.adaptiveThreshWinSizeMin = 4
            cam1_param.adaptiveThreshWinSizeMax = 25
            cam1_param.adaptiveThreshWinSizeStep = 3

            cam1_param.cornerRefinementWinSize = 6
            cam1_param.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
            # cam1_param.cornerRefinementMaxIterations = 10
            # cam1_param.cornerRefinementMinAccuracy = 0.01

            #Compute detection and pose estimation using camera 1
            pos_cam, quat, cam_att_vec, img = aruco_detection(image, dictionary, cam1_param, rvecs, tvecs, mtx, dist)

            

            #Check if there is information provided by camera 1
            if quat is not None and cam_att_vec is not None:

                #Position Correction
                # pos_cam[0,0] = 0.751*pos_cam[0,0] - 0.105
                # pos_cam[1,0] = 0.759*pos_cam[1,0] - 0.106
                # pos_cam[2,0] = 0.878*pos_cam[2,0] + 0.106

                # print(pos_cam.T)
                pos_cam[0,0] = pos_cam[0,0]*data['theta x'][0] + data['theta x'][1]
                pos_cam[1,0] = pos_cam[1,0]*data['theta y'][0] + data['theta y'][1]
                pos_cam[2,0] = pos_cam[2,0]*data['theta z'][0] + data['theta z'][1]

                # #Z factor
                # pos_cam[0,0] = pos_cam[0,0]*data['theta x'][0] + data['theta x'][1]
                # pos_cam[1,0] = pos_cam[1,0]*data['theta y'][0] + data['theta y'][1]

                #Store the pose and camera vector in messages
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
                pose.pose.position.x = pos_cam[0,0]
                pose.pose.position.y = pos_cam[1,0]
                pose.pose.position.z = pos_cam[2,0]

                cam_vec.vector.x = cam_att_vec[0,0]
                cam_vec.vector.y = cam_att_vec[1,0]
                cam_vec.vector.z = cam_att_vec[2,0]

                #Store camera 1 flag which indicate if there is new data
                flag_cam1.data = True
                
            else:
                
                #Store camera 1 flag which indicate if there is no new data
                flag_cam1.data = False
            
            #Store stamptime and frame ID for pose and camera attitude vector messages
            pose.header.frame_id = 'Pose Camera 1'
            pose.header.stamp = rospy.Time.now()

            cam_vec.header.frame_id = 'Camera 1 Attitude Vector'
            cam_vec.header.stamp = rospy.Time.now()

            #Publish pose and camera vector
            pose_pub.publish(pose)
            cam_vec_pub.publish(cam_vec)
            cam1_info_pub.publish(flag_cam1)

            #Show processed image
            cv.imshow('Camera 1', img)

        #Verify if we got some information from camera 2 sensor
        if mtx2 is not None and dist2 is not None:
            
            #Initialize the detector parameters using defaults values
            cam2_param = cv.aruco.DetectorParameters_create()
            cam2_param.adaptiveThreshWinSizeMin = 4
            cam2_param.adaptiveThreshWinSizeMax = 25
            cam2_param.adaptiveThreshWinSizeStep = 2

            cam2_param.cornerRefinementWinSize = 6
            cam2_param.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
            # cam2_param.cornerRefinementMaxIterations = 10
            # cam2_param.cornerRefinementMinAccuracy = 0.01

            #Compute detection and pose estimation using camera 2
            pos_cam2, quat2, cam2_att_vec, img2 = aruco_detection(image2, dictionary, cam2_param, rvecs, tvecs, mtx2, dist2)


            #Check if there is information provided by camera 2
            if quat2 is not None and cam2_att_vec is not None:

                #Position Correction
                # pos_cam2[0,0] = 0.788*pos_cam2[0,0] + 0.15
                # pos_cam2[1,0] = 0.781*pos_cam2[1,0] + 0.135
                # pos_cam2[2,0] = 0.854*pos_cam2[2,0] + 0.11

                pos_cam2[0,0] = pos_cam2[0,0]*data['theta x2'][0] + data['theta x2'][1]
                pos_cam2[1,0] = pos_cam2[1,0]*data['theta y2'][0] + data['theta y2'][1]
                pos_cam2[2,0] = pos_cam2[2,0]*data['theta y2'][0] + data['theta y2'][1]

                #Store the pose and camera vector in messages
                pose2.pose.orientation.x = quat2[0]
                pose2.pose.orientation.y = quat2[1]
                pose2.pose.orientation.z = quat2[2]
                pose2.pose.orientation.w = quat2[3]
                pose2.pose.position.x = pos_cam2[0,0]
                pose2.pose.position.y = pos_cam2[1,0]
                pose2.pose.position.z = pos_cam2[2,0]
                
                cam2_vec.vector.x = cam2_att_vec[0,0]
                cam2_vec.vector.y = cam2_att_vec[1,0]
                cam2_vec.vector.z = cam2_att_vec[2,0]
                
                #Store camera 2 flag that indicates if there is new data
                flag_cam2.data = True
            
            else:
                
                #Store camera 2 flag that indicates if there is no new data
                flag_cam2.data = False
            
            #Store stamptime and frame ID for pose and camera attitude vector messages
            pose2.header.frame_id = 'Pose Camera 2'
            pose2.header.stamp = rospy.Time.now()

            cam2_vec.header.frame_id = 'Camera 2 Attitude Vector'
            cam2_vec.header.stamp = rospy.Time.now()
            #Publish pose and camera vector
            pose2_pub.publish(pose2)
            cam2_vec_pub.publish(cam2_vec)
            cam2_info_pub.publish(flag_cam2)

            #Show processed image
            cv.imshow('Camera 2', img2)


        cv.waitKey(1) 

        rate.sleep()


if __name__ == '__main__':
    try:
        computer_vision_publisher()
    except rospy.ROSInterruptException:
        pass