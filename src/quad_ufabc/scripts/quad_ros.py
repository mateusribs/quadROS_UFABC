import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Float64MultiArray, Float32
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as rot


class quad_robot:
    def __init__(self, name):
        self.name = name

        # Rate of the controller
        self.rate = rospy.Rate(100)

        self.motor_vel = Float64MultiArray()

        self.PoseSub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_read, queue_size=1)
        self.jointSub = rospy.Subscriber('/'+self.name+'/joint_states', JointState, self.joint_read, queue_size=1)
        self.reset_command = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.vel_pub = rospy.Publisher(name + '/joint_motor_controller/command', Float64MultiArray, queue_size=1)
        
        self.attitude_euler = np.zeros(3)
        self.attitude_quat = np.zeros(4)
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        self.angular_vel = np.zeros(3)
        self.reset_command = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def joint_read(self, data):
        self.data = data
        self.prop_velocity = data.velocity

    def joint_pub(self, data):
        msg = JointState()
        msg.name = ['joint_back_left_prop', 'joint_back_right_prop', 'joint_front_left_prop', 'joint_front_right_prop']
        msg.velocity = [0, 0, 0, 0]
        self.jointPub.publish(msg)

    def step(self, motor_vel):
        self.motor_vel.data = motor_vel
        self.vel_pub.publish(self.motor_vel)


    def pose_read(self, data):
        """
        Stores the position and attitude data on the class
        """
        ind = data.name.index(self.name)
        orientationObj = data.pose[ind].orientation
        positionObj = data.pose[ind].position
        velocityObj = data.twist[ind].linear
        angularvelocityObj = data.twist[ind].angular
        self.position = np.array([positionObj.x, positionObj.y, positionObj.z])
        self.attitude_quat = np.array([orientationObj.w, orientationObj.x, orientationObj.y, orientationObj.z])
        self.attitude_euler = rot.from_quat(self.attitude_quat).as_euler('xyz')
        self.velocity = np.array([velocityObj.x, velocityObj.y, velocityObj.z])
        self.angular_vel = np.array([angularvelocityObj.x, angularvelocityObj.y, angularvelocityObj.z])
    
    def reset(self):
        """
        Resets robot to initial position
        """
        command = ModelState()
        command.model_name = self.name
        location = Pose()
        # Location Reset
        self.init_pose = np.random.randn(2)/2

        location.position.x = 0.6
        location.position.y = 0
        location.position.z = 0

        # Orientation reset
        self.init_orientation = rot.from_euler('xyz', [0, 0, 0]).as_quat().flatten()
        location.orientation.x = self.init_orientation[0]
        location.orientation.y = self.init_orientation[1]
        location.orientation.z = self.init_orientation[2]
        location.orientation.w = self.init_orientation[3]

        command.pose = location
        self.reset_command(command)

        self.position = np.concatenate((self.init_pose, np.zeros(1)))
        self.attitude_euler = np.array([0, 0, 0])
        self.attitude_quat = self.init_orientation
        self.velocity = np.zeros(3)
        self.angular_vel = np.zeros(3)

        self.step([0, 0, 0, 0])
        # rospy.sleep(5)

