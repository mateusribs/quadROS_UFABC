FROM osrf/ros:noetic-desktop-full

LABEL maintainer='mateus.ribeiro'
LABEL version='0.01'
LABEL description='Test for quadrotor UFABC'

RUN apt -y update && apt install -y git python3-pip libqt5gui5 ros-noetic-ros-control ros-noetic-ros-controllers \
                     python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
                     && rm -rf /var/lib/apt/lists/* 

ENV QT_DEBUG_PLUGINS=1
WORKDIR /home
COPY requirements.txt requirements.txt
COPY setup_env.sh setup_env.sh

WORKDIR /home/src

RUN bash /home/setup_env.sh

COPY src /home/src/

WORKDIR /home




