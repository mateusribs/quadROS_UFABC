# quadROS UFABC

This repository aims to maintain the research made in my master's thesis. The title of the project is "Position and Attitude Estimation of a Quadrotor Using Computer Vision", where the main task is to accomplish indoor navigation in a GPS-denied environment using computer vision techniques. The state estimation was made using Error State Kalman Filter [1] and for the quadrotor's control, it was implemented a cascade PID. The simulations are available in [Video Simulation](https://youtu.be/_FUcOvvbstQ).


# Installation

## Docker (Recommended)

Clone this repository and follow the instructions:

1 - Build a Docker image:

```
~$ docker build . -t quad_ufabc:latest
```
2 - Run shell script to create the Docker container:

```
~$ sudo bash start_container.bash
```

3 - Execute the commands located on **Running application** section of this repository.

## Linux (Ubuntu 20.04 LS)

The ROS version used in this work is Noetic. The official installation guide can be accesed on:[ROS Noetic Installation](http://wiki.ros.org/noetic/Installation/Ubuntu). It is reccommended that in 1.4 Installation topic, the choice of the option **Desktop-Full Install**, because it already install all the dependencies, including the Gazebo simulator. 

After ROS/Gazebo installation, it is necessary create a workspace. Open the terminal on Ubuntu, and follow the instrunctions:

- Install some dependencies of ROS:

```
~$ sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers -y
```

- Create a work directory:

```
~$ mkdir /path/to/work/directory
```

- On the created directory, create another folder called **src** and initialize the workspace:

```
~$ mkdir /path/to/work/directory/src
~$ cd /path/to/work/directory/src
~$ source /opt/ros/noetic/setup.bash
~$ catkin_init_workspace 
```

- Compile workspace:

```
~$ cd ..
~$ catkin_make
```
- Configure the .bashrc:

```
~$ echo "source /path/to/work/directory/devel/setup.bash" >> ~/.bashrc
~$ source ~/.bashrc
```

- Clone this repository, copy and paste the **src** folder into the work directory.

- Create a virtual environment (in this case, we will use __venv__ from Python, but feel free to use another one):

```
~$ python3 -m venv venv
~$ source venv/bin/activate
```

- On cloned repository, run the python dependencies installation using __pip__:
```
~$ pip install -r requirements.txt
```

## Running application

Initialize Gazebo:

```
~$ roslaunch quad_ufabc quad.launch
```

Run simulation

```
~$ rosrun quad_ufabc main.py
```


## Referências

[1] SOLÀ, J. Quaternion kinematics for the error-state kalman filter. 2017.
