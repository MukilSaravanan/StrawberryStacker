# Task 3: Pick and Place

## Task 3.1

## Overview

- Picking and placing in gazebo environment using drone.
- Setpoint Navigation in offboard mode.

## Resouces
We highly recommend you to go through [PX4 official documentation](https://docs.px4.io/master/en/flight_modes/offboard.html), to get details about offboard mode.

## Problem Statement
- The gazebo world consist of a drone and a strawberry box.
- You have to put the drone in Offboard mode and publish the positions of the drone as setpoints.
- The location of the box is 3m 0m 0m, the drone needs to do the following:
    - Takeoff and the initial position to the height of 3m
    - Go to the coordinate 3m 0m 3m
    - Land on the box and pick the box
    - Takeoff at the height of 3m and go to 3m 3m 3m
    - Land at 3m 3m 0m and drop the box
    - Takeoff again to height of 3m
    - Head back to the start position ie 0m 0m 3m
    - Finally land the drone at 0m 0m 0m and then disarm
- After landing on the box, you need to check that the drone is above the box in the allowable range to pick the box. To do that, you need to subscribe to the rostopic _/gripper_check_. If the value is _true_, you can now pick the box and if the value is _false_,the drone is not correctly placed and you need to correct the position.
- If the result of _/gripper_check_ is _true_, to pick the box, you need to call the rosservice _/activate_gripper_ and pass the value _true_ to attach the box. If the box is attached, you will get a result from the rosservice as _true_. If the result is _false_, which means the drone is not correctly placed over the box.
- To detach the box, you need to use the same rosservice _/activate_gripper_ and pass the value _false_. This will detach the box from the drone.

## Procedure
- Launch the Gazebo world by typing the following command in a terminal
```bash
roslaunch task_3 task3_1.launch
```
- Set the following PX4 Parameters in the QGC GUI:
    - COM_RCL_EXCEPT = 4
    - COM_DISARM_LAND = -1
- Once the simulation window launches, you should see a drone and a box in the gazebo environment.
- Run your python script in a separate terminal to start sending the setpoints and navigate
the drone.

## Result

[![task3_1.mp4](https://img.youtube.com/vi/YIRRQZaZVXk/0.jpg)](https://www.youtube.com/watch?v=YIRRQZaZVXk)

## Task 3.2

## Overview

- Picking and placing in gazebo environment using drone.
- Setpoint Navigation in offboard mode.
- Interfacing of ArUco detection and setpoint navigation

## Resources
- To get an overview about offboard mode and setpoint navigation you can go through the [px4 official documentation](https://docs.px4.io/master/en/flight_modes/offboard.html).

## Problem Statement
- The gazebo world consist of a drone and a strawberry box with ArUco marker on it.
- You have to put the drone in Offboard mode and publish the positions of the drone as setpoints.
- The drone needs to do the following:
Takeoff at the initial position to the height of 3m
  - Start moving towards the final setpoint 9m 0m 3m
  - While travelling scan for the strawberry box with ArUco marker. As soon as the drone reaches above the box, land on it and pick up the box.
  - Again takeoff to the height of 3m and continue to move towards 9m 0m 3m
  - Land at 9m 0m 0m and drop the box
  - Takeoff again to height of 3m
  - Head back to the start position ie 0m 0m 3m
  - Finally land the drone at 0m 0m 0m and then disarm
- After landing on the box, you need to check that the drone is above the box in the allowable range to pick the box. To do that, you need to subscribe to the rostopic _/gripper_check_. If the value is _true_, you can now pick the box and if the value is _false_,the drone is not correctly placed and you need to correct the position.
- If the result of _/gripper_check_ is true, to pick the box, you need to call the rosservice _/activate_gripper_ and pass the value _true_ to attach the box. If the box is attached, you will get a result from the rosservice as _true_. If the result is _false_, which means the drone is not correctly placed over the box.
- To detach the box, you need to use the same rosservice _/activate_gripper_ and pass the value _false_. This will detach the box from the drone.

## Procedure
- Launch the Gazebo world by typing the following command in a terminal
```bash
roslaunch task_3 task3_2.launch
```
- Once the simulation window launches, you should see a drone and a box in the gazebo environment.
- Run your python script in a separate terminal to start sending the setpoints and navigate the drone.

## Result

https://user-images.githubusercontent.com/47444101/179043970-8a60ed13-dfcf-4a49-a357-10e476f7a532.mp4

### Quick Navigation
- [Up] [Task 2 - Getting Started with PX4](../task_2/)
- [Down] [Task 4 - Pick and Place Using Multiple Drones](../task_4/)

