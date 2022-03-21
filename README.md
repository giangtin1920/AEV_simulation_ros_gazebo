# AEV_simulation_ros_gazebo

Self Driving Car simulation in a world using ROS and Gazebo.

## Building the package

### 1. Path for ROS workspace

    source ~/catkin_ws/devel/setup.bash

### 2. Building workspace catkin

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    
### 3. Cloning packages

    cd ~/catkin_ws/src
    git clone https://github.com/giangtin1920/AEV_simulation_ros_gazebo.git
    
### 4. Installing Dependencies

    sudo apt-get update
    sudo apt-get install ros-melodic-laser-assembler ros-melodic-laser-geometry ros-melodic-pcl-ros ros-melodic-cmake-modules
    sudo apt-get install libpcl1
    source /opt/ros/melodic/setup.bash

### 5. Compile the workspace

    cd ~/catkin_ws
    catkin_make

## Running Simulation

#### Rviz:

    roslaunch carsim_gazebo rviz.launch

#### Gazebo:

    roslaunch carsim_gazebo carsim.launch

#### Keyboard Teleop:

    rosrun carsim_gazebo teleop.py

## Acknowledgments and References

https://github.com/duthades/car_simulation_ros_gazebo

https://github.com/saharshleo/obstacleAvoidanceRobot

https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/

http://gazebosim.org/tutorials/?tut=ros_urdf

http://gazebosim.org/tutorials/?tut=build_robot

https://www.youtube.com/watch?v=mFTkN5v4Jzc

