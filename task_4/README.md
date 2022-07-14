# Task 4: Pick and Place Using Multiple Drones

## Overview
This task includes:
- Operating multi drones in the gazebo environment
- Pick and place using multi drones

## Resources

- To get an overview about offboard mode and setpoint navigation you can go through the [PX4 official documentation](https://docs.px4.io/master/en/flight_modes/offboard.html)
- To get familiar with the Multi-Vehical system of PX4 you can go through the [Official PX4 documentation](https://docs.px4.io/master/en/simulation/multi-vehicle-simulation.html)

## Prerequisites

Following extra package is required. Install it using the following command in the terminal.
```bash
sudo apt install xmlstarlet
```

## Problem Statement

- In this task you can see a complete Strawberry farm setup, with 15 rows of strawberry plants numbered from 1-15, strawberry boxes of 2 colors ie. red and blue, 2 trucks (one blue and one red) on the side of the farm and 2 drones named __edrone1__ and __edrone2__ on their respective starting points.

- The two different kinds of strawberry boxes have different significance. The premium quality strawberries are packed in blue colored boxes and normal quality strawberries are kept in red colored boxes. The aim here is to deliver red colored strawberry boxes on red truck and blue colored boxes to blue truck.

- The farm consists of rows of strawberry plants followed by empty row spaces where the strawberry boxes will be kept as and when they are packed. This is the same area where the eDrone has to come and land on the boxes to pick them up. These empty rows are spaced 4m apart, and the location of the 1st row is the same as the starting position of edrone0 and the location of the last row is same as the starting location of __edrone1__. Since the distance between each row and the location of the 1st row is known to you, you can calculate the coordinate of any row.

- The delivery trucks have a grid drawn on them where each cell is the desired location for the strawberry box to be placed. The exact location of 1st cell is given to you and the length and breadth of each cell is also given to you. So you can calculate the exact coordinates of any given cell.

    > Note: The delivery strategy is upto you whether you want to stack boxes on top of each other or you want to spread your boxes evenly. Note that for now there are less boxes in the scene but the number boxes will increase in the future tasks, so you might need to stack them as per the situation.

- After you have starting coordinates of each row, you will see boxes being spawned in a pseudo random manner in the gazebo environment, the row number where the box is being spawned is given below. There are two types of boxes, blue and red, with aruco marker of different aruco id.

- You have to determine the exact position of the box (you are provided only the row number, use the strategy used in task 3.2 to locate the box in a row) pick it and deliver it to one of the trucks.

- Setpoints that are required:
    - Home position of __edrone0__ : -1m 1m
    - Home position of __edrone1__ : -1m 61m
    - Space between 2 rows : 4m
    - 1st cell of blue truck : 13.85m -7.4m 1.7m
    - 1st cell of red truck : 56.5m 64.75m 1.7m
    - Length of cells on truck : 0.85m
    - Width of cells on truck : 1.23m
    - Row number of Boxes:
        - Box 1: 5
        - Box 2: 7
        - Box 3: 8
        - Box 4: 13
    > Note: All the coordinates are given in the global reference, so if you want the eDrone1 to go to red truck, subtract the initial coordinates from the destination to get your setpoint for the eDrone1. eg. setpoint for eDrone1 to reach blue truck = [13.85 - (-1) m , -7.4 - 1 m , 1.7 - 0 m] = [14.85, -8.4m, 1.7m]. Similarly you can calculate setpoint for any position since you have starting position of eDrones in global reference.

You can add your own setpoints to make the navigation of drones smooth.

- Maintain minimum height of the eDrone’s at least 3m from ground while cruising (the higher you go, more context is captured in the camera but at the same time more time is taken to travel, so optimize your cruising height).

- Strategy to deploy two drones to complete the task is completely up to your creativity, you can use:

- One drone to scan and send locations of box and second for pick and place
Scanning and pick and place from both the drones simultaneously
Or any other method that might reduce the time of run.

__Note:__ _This is the time based task. That is the task will need to be finished in the least amount of simulation time._

## Procedure


- Launch the Gazebo world by typing the following command in a terminal
```bash
roslaunch task_4 task4.launch
```

- Once the simulation window launches, you should see complete farm setup in gazebo environment.

- Since there are multiple eDrones in the scene now, the names of all the rostopics have changed. Each rostopic has a prefix as the eDrone number. For eg. ```/mavros/setpoint_position/local``` becomes ```/edrone1/mavros/setpoint_position/local``` for edrone1 and ```/edrone2/setpoint_position/local``` for edrone2. This applies to all other _rostopics_ including ```/gripper/check```, ```/camera/image_raw``` etc. This also applies to _rosservices_. For exact list of ```rostopics``` and ```rosservices``` you can run the command rostopic list and rosservice list after launching the file.

- For picking up the boxes with eDrones, you need to call the appropriate rosservice for each eDrone and similarly for dropping the boxes.

- Run your python script in a separate terminal to start the task.

## Result

### Quick Navigation
- [Up] [Task 3 - Pick and Place](../task_3/README.md)
- [Down] [Task 5 - Theme Implementation](../task_5/README.md)
