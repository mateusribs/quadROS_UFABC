#!/usr/bin/env python3
import rospy
import roslaunch


def start_control():

    rospy.loginfo('Starting Quadrotor Control Node...')

    package = 'quad_ufabc'
    executable_control = 'control_node.py'

    node_control = roslaunch.core.Node(package, executable_control)

    launch_control = roslaunch.scriptapi.ROSLaunch()

    launch_control.start()

    controller = launch_control.launch(node_control)

    if controller.is_alive():
        print('Control Initialized')


def start_cv():

    rospy.loginfo('Starting Computer Vision Node...')

    package = 'quad_ufabc'
    executable_cv = 'computer_vision_node.py'

    node_cv = roslaunch.core.Node(package, executable_cv)

    launch_cv = roslaunch.scriptapi.ROSLaunch()

    launch_cv.start()

    comp_vision = launch_cv.launch(node_cv)

    if comp_vision.is_alive():
        print('Computer Vision Initialized')

def start_estimator():

    rospy.loginfo('Starting Estimation Node...')

    package = 'quad_ufabc'
    executable_estimation = 'estimation_node.py'

    node_estimation = roslaunch.core.Node(package, executable_estimation)

    launch_estimation = roslaunch.scriptapi.ROSLaunch()

    launch_estimation.start()

    estimation = launch_estimation.launch(node_estimation)

    if estimation.is_alive():
        print('Estimation Initialized')


def main():

    rospy.init_node('Main')
    start_cv()
    rospy.sleep(1)
    start_control()
    start_estimator()
    
    rospy.spin()

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException():
        pass
