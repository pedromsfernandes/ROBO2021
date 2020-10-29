# Reactive Robot

## Pre-Requisites

* [Ubuntu 16](https://releases.ubuntu.com/16.04/)
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [stdr_simulator](http://wiki.ros.org/stdr_simulator/Tutorials/Set%20up%20STDR%20Simulator#Get_STDR_Simulator_from_Github)

## Usage

1. Make sure you have [configured the catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace).

1. Create package for the project

```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg reactive_robot
```

2. Copy the files into the newly created folder

3. Copy the files in the `stdr_files` folder into their respective folders in the `stdr_simulator` package.

4. Build the package

```bash
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

## Run

1. Launch the map

```bash
$ roslaunch stdr_launchers reactive_robot.launch
```

2. Deploy the robot

```bash
$ rosrun reactive_robot stdr_wall_following robot0 laser_0
```