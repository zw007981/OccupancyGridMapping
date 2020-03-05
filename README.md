# OccupancyGridMapping
Simulation for turtlebot exploring and mapping.


**Contents**
* [Introduction](#introduction)
* [Running](#Running)

## Introduction

This repository is a simulation for exploring and mapping unknown environment using turtlebot. This project is tested with **Ubuntu 18.04 LTS** and **ROS Melodic**.  Codes are developed in **Python 2.7**.

## Running

1. Save this package into catkin_ws/src/.

2. Open a new world in Gazebo and start rviz.

`roslaunch simulation test.launch`.

3. Start exploring.

In a new terminal, run `rosrun simulation test.py`.

4. Save the map.

The map generated will be saved in the folder "images" as "map.png" automatically, one of the maps generated is shown below:

<img src="https://github.com/zw007981/simulation/blob/master/images/map.png" width="450">

but you can still save the map in rviz by running `rosrun map_server map_saver`, as shown in the figure below:

<img src="https://github.com/zw007981/simulation/blob/master/images/map_rviz.png" width="450">
