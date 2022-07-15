# Task 2: Getting Started with PX4

## Task 2.1

## Overview
- Getting started with px4 firmware
- Learning about various modes
- Sending Waypoints to px4 via mavros

## Problem Statement
- You need to make the drone in Gazebo to follow 4 waypoints in the mission mode of px4
- Create a rosnode named waypoint_Mission in a python script, which will set the waypoints to be sent to px4
- You need to call the rosservice /mavros/cmd/arming to arm and /mavros/set_mode to set mode of the drone to mission mode
- Then you have to call the rosservice /mavros/mission/push and /mavros/mission/pull to Request parameter from device (or internal cache) and send parameters from ROS to FCU respectively.
- The waypoints are as follows:
    - Takeoff at the home position to 10 meters
    - Go to 19.134641, 72.911706, 10
    - Go to 19.134617, 72.911886, 10
    - Go to 19.134434, 72.911817, 10
    - Go to 19.134423, 72.911763, 10
    - Land at the last coordinate

## Resources

### Install QGroundControl
- On the command prompt enter:
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```
- Logout and login again to enable the change to user permissions.
- Download [QGroundControl.AppImage](https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage)
```bash
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
```
- Install (and run) using the terminal commands:

```bash
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```

### Learning Resources
- Highly recommended to go through [px4 official documentation](https://docs.px4.io/v1.12/en/development/development.html) (following till Modules & Commands will be enough for this task) and for mavros go through [mavros px4 documentation](https://docs.px4.io/master/en/ros/ros1.html) before proceeding with the actual task as the learning curve might be steep in the start.

- This task will require the knowledge of _custom modes_, _custom messages_ and _mavros integration_

## Problem Statement
- You need to make the drone in Gazebo to follow 4 waypoints in the __mission__ mode of px4
- Create a rosnode named ```waypoint_Mission``` in a python script, which will set the waypoints to be sent to px4
- You need to call the _rosservice_ ```/mavros/cmd/arming``` to arm and ```/mavros/set_mode``` to set mode of the drone to __mission__ mode
- Then you have to call the _rosservice_ ```/mavros/mission/push``` and ```/mavros/mission/pull``` to Request parameter from device (or internal cache) and send parameters from ROS to FCU respectively.
- The waypoints are as follows
    - Takeoff at the home position to 10 meters
    - Go to 19.134641, 72.911706, 10
    - Go to 19.134617, 72.911886, 10
    - Go to 19.134434, 72.911817, 10
    - Go to 19.134423, 72.911763, 10
    - Land at the last coordinate

## Procedure
- Launch the Gazebo world by typing the following command in a terminal
```bash
roslaunch task_2 task2_1.launch
```
- Once the simulation window launches, you should see a drone in the gazebo environment.
- Run your python script in a separate terminal to start sending the waypoints and navigate the drone.

## Result
https://user-images.githubusercontent.com/72087882/142737636-8195dc1c-8dc4-473a-9b19-a8226289157e.mp4

## Task 2.2

## Problem Statement
- You need to put the drone in Offboard mode and make a square of 10m x 10m.
- Takeoff from the ground and give the first setpoint as 0m,0m,10m
- Give the next setpoint as 10m,0m,10m followed by 10m,10m,10m followed by 0m,10m,10m and then back to 0m,0m,10m
- Finally land at the home position and disarm
Keep the velocity of drone 5 m/s

## Resources
Recommend to go through [PX4 Official Documentation](https://docs.px4.io/master/en/flight_modes/offboard.html), to get details about offboard mode.

## Procedure
- Launch the Gazebo world by typing the following command in a terminal
```bash
roslaunch task_2 task2_2.launch
```
Once the simulation window launches, you should see a drone in the gazebo environment.
- Run your python script in a separate terminal to start sending the setpoints and navigate the drone.

## Result

https://user-images.githubusercontent.com/72087882/144624247-956f7848-01a4-4b9b-912f-a3ed716017ef.mp4

### Quick Navigation
- [Up] [Task 1 - Getting Started with ArUco and ROS](../task_1/)
- [Down] [Task 3 - Pick and Place](../task_3/)
