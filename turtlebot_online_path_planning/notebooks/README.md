# On-line path planning

**This project belongs to the Universitat de Girona. It is forbidden to publish this project or any derivative work in any public repository.**

This lab will combine the concepts of occupancy grid mapping and path planning seen in previous sessions.
The main goal of this lab is to move a simulated or real Turtlebot-like robot from its current position to a goal position avoiding the obstacles in the environment.

## Installation

The following element are necessary to run this package. Remember that after installing a new package, you have to source the `devel/setup.bash` file.

### Gazebo Simulator

To install Gazebo follow these instructions (from [gazebosim](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)):

```bash
# Add a new repository
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Set up the keys
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Update debian database
sudo apt update

# Install Gazebo
sudo apt install gazebo11

# Check that gazebo is correctly installed
gazebo
```

### Install gazebo ROS packages

Install the following package.

```bash
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

You can run the following command to check if the installation was successful:

```bash
roslaunch gazebo_ros empty_world.launch
```

### Turtlebot3


 [TurtleBot 3](https://www.turtlebot.com/) is the third version of ROS standard platform robot. There are 3 versions of robots (i.e., burger, waffle and waffle_pi). There are several ROS package related with turtlebot 3 but all of them can be installed installing only the following meta package:

```bash
sudo apt update
sudo apt install ros-noetic-turtlebot3*
```

You can run the following command to check if the installation was successful:

```bash
export TURTLEBOT3_MODEL="waffle"
roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
```

### Octomap Server

The Octomap server is a ROS package that provides a 3D occupancy grid map of the environment. It is used to generate the 2D occupancy grid map that will be used by the path planner. To install it, run the following command:

```bash
sudo apt install ros-noetic-octomap*
```

You can run the following command to check if the installation was successful:

```bash
roslaunch roslaunch turtlebot_online_path_planning gazebo.launch
```

## Pre-lab

In this lab we are going to implement an on-line path planning algorithm. The first thing you should do is to complete the Notebook [state_validity_checker.ipynb](notebooks/state_validity_checker.ipynb).

## Online path planning architecture

To implement a path planner that can run in an unknown environment in real time several modules are required:

* A map server that publishes the occupancy grid map of the environment that has been observer till this moment.
* A path planner that computes the path from the current position to the goal.
* A controller that moves the robot along the path.

We are going to use the following algorithms for all these nodules:

* Map server: An occupancy grid map, the Octomap server, will be used. Check the documentation [here](https://wiki.ros.org/octomap_server).
* Path planner: Several path planning algorithms (i.e., A*, RRT, ...) have been implemented. Here, you can use RRT, or any similar algorithm.
* Controller: A controller that moves a robot to a specific position was already been implemented in the `turtlesim` lab.

The following figure shows the relations between all these modules.

<div style="text-align: center">
    <img src='./imgs/online_planning_architecture.png' width='600'/>
</div>

As you can see, the center piece is the `planner`, which is responsible for finding a path from the current position to the goal. The connection with the occupancy map is done through the `state validity checker` module that for each configuration examined by the `planner`, it check if there is a collision with the last available `map`. The `controller` takes the list of configurations computed by the `planner` to reach the goal (i.e., the `path`) and it is responsible for moving the robot along them. Every time that the `map server` publishes a new map, the `state validity checker` is also updated and the planned path has to be checked as new obstacles can make it invalid.

## Implementation

Once introduced the main modules that must be implemented, lets see in more detail how to implement each of them. It is very important to make each module ROS agnostic. It means that ROS specific code do not has to be included in the following modules. We will separate our code in a file containing all the functionalities of our modules and another file, a ROS node, that will make use of all these modules. This file (i.e., the ROS node) will contain all ROS related aspects: subscribe and publish all required messages, timers, configuration files, ...

### State validity checker

The Octomap server publishes a 2D occupancy grid (`nav_msgs/OccupancyGrid`) map throug the topic `/projected_map` that is updated every time a new scan is received by the Octomap server. This 2D grid will be used by the `state validity checker` to check if a configuration is valid or not.

The `state validity checker` has to implement at least 3 functions: `set`, `isValid`, and `checkPath`.

#### `set(data, resolution, origin)`: 

This function is called by the ROS node when a new occupancy grid is received. It is used to update the map that will be used to check the state validity.

* `data`: is a `int` array. To transform the `nav_msgs/OccupancyGrid/data` to a 2D matrix it has to be reshaped to `nav_msgs/OccupancyGrid/info/width x nav_msgs/OccupancyGrid/info/height` and transposed
* `resolution` is the resolution of the map (meters per cell).
* `origin` is the position of the origin of the map (in meters).

#### `is_valid(x, y, distance=0.1)`:

This function checks if the position `(x, y)` defined in meters is valid or not. To do it, it is required to transform the position from meters to the cell index that the position belongs to. If `distance` is greater than 0, it is required to check if the cells at distance `distance` from the `(x, y)` position are also free or not. Consider the map `resolution` to see how many cells around `(x, y)` have to be checked. The function returns `True` if all checked cells are free and `False` otherwise.

To transform from Cartesian position in meters ($Cartesian_p$) to cell index $Cell_p$, the following formula is used:

$$
 Cell_p = \frac{Cartesian_p - Map_{origin}}{Map_{resolution}}
$$

If the requested position is outside the map, the function must return `True` if the parameter `is_unknown_valid` is defined as `True` and `False` otherwise. If `is_unknown_valid` argument is set to `False`, it will not be possible to send the robot to unknow positions. If it is set to `True`, planning is more difficult because when the robot reaches a position closer to the unknown cells, the map will be updated and the robot will known if the requested position is free or occupied. Therefore, to check if the path is still valid, it will necessary to call the method `check_path`. The algorithm must work with `is_unknown_valid` set at `True` and set a `False`.

#### `check_path(path, min_dist)`:

This function checks if a `path` generated by the path planner is still valid or not after an update in the occupancy map. It is especially important to check that when there are dynamic obstacles or when `is_unknown_valid` is set to `True`. If the `path` is invalid, a new `path` must be computed.

While the path is just a list of configurations (i.e., $x$, $y$ positions), the `check_path` function must check not only if these positions are valid but also that the robot can move along them. A simple solution is to discretize the *segment* between two configurations using the `min_dist` parameter and check if each intermediate configuration is valid using the `is_valid` function.

### Path Planner

The path planner function needs a `start` and a `goal` position (i.e., `x, y` in meters) and access to the `state validity checker` to compute a valid `path` from the `start` to the `goal`. The `start` position can be the current position of the robot.

Only the `compute_path` function must be implemented.

#### `compute_path(start, goal, state_validity_checker, dominion)`:

Using any path planning algorithm (i.e., RRT, RRT*, PRM...) and the state validity checker object, this function computes a valid `path` from the `start` to the `goal`. The `path` is a list of `(x, y)` positions in meters.
To implement it, you should first fill the `RRT` class provided in the skeleton with the code you have already implemented in the previous lab. Feel free to change this task for a `PRM` or `RRT*` implementation.


The path planner will need access to the `state_validity_checker` while planning. It also will need to know the `dominion` of the environment. The `dominion` is a 2D array containing the minimum bound and maximum bound where the planner will search for a solution (i.e., [$min_x$, $max_x$, $min_y$, $max_y$]).


### Controller

Once a `path` is computed, the controller is responsible for moving the robot along it. The controller only has to implement one function that must be called at a constant frequency if a valid `path` exists: `move_to_point`.

#### `move_to_point(x, y, theta, x_goal, y_goal)`:

The `moveToPoint` function computes the desired forward velocity ($\nu_d$) and desired angular velocity ($w_d$) that the robot must apply to move from the current position ($x$, $y$, $\theta$) to the goal position ($x_{goal}$, $y_goal$). Because we want the robot to stay over the path as much as possible, the controller must compute first the desired angle between the current position and the goal ($\theta_d$) and only when the desired angle is almost equal than the current one (i.e., $\theta_d \approx \theta$) move forward with a velocity proportional to the distance with respect to the goal. When the goal position is reached with some tolerance, this waypoint has to be removed from the `path` and the next configuration in the `path` will become the new goal.

$$
\theta_d = \tan^{-1}\frac{y_g - y}{x_g - x} 
$$

$$
w_d = K_h \text{wrap\_angle}(\theta_d - \theta)
$$

$$
\nu_d = 
\begin{cases}
    K_p \sqrt{(x_g - x)^2 + (y_g - y)^2} & \text{if~} \theta_d \approx \theta \\
    0 & \text{otherwise}
\end{cases}
$$

Where $K_v > 0$ and $K_h > 0$.

## Implementation

Following the skeleton provided in the file `utils_lib/online_planning.py`, implement a class for the `state_validity_checker`, the functions for the `path_planner` and the `controller` modules as well the necessary classes functions for the planning algorithm being used (e.g., the `RRT` class).

Complete the ROS node called `online_planning_node.py` that subscribes to the topics `/projected_map` from the Ocotmap server, `/odom` from the turtlebot3 pose, and `/move_base_simple/goal` published by the `rviz` visualizer. This node also has to publish the topics `/cmd_vel` to move the turtlebot3 as well as a `visualization_msgs/Marker` in the topic `path` to visualize the computed `path` in `rviz`. 

The `online_planning_node.py` will import the functions previously defined and implement the following logics:

* When a `/projected_map` message is received:
  * The state validity checker must be updated using the `set` function.
  * If there is a `path` already computed, its validity must be checked using the `check_path` function. If the `path` is not valid anymore, a new `path` must be computed to the same goal using the `compute_path` function.
* Every time that a `/move_base_simple/goal` message is received the planner will be asked to plan a new `path` using the `compute_path` function.
* At a constant frequency, e.g. at 10Hz, if there is a valid `path` computed, the `move_to_point` function must be called with the current pose of the robot (i.e., $x$, $y$, $\theta$ obtained from the `/odom` topic) and the first configuration in the `path` as a goal. If this position is reached with some tolerance, this configuration must be removed from the `path` and the next configuration will become the new goal for the `move_to_point` function. The $\nu_d$ and $w_d$ computed by the `move_to_point` function have to be published as a `geometry_msgs/Twist` in the `cmd_vel` topic in order to move the turtlebot3. If there is no valid `path`, a `geometry_msgs/Twist` message with zero velocity must be published instead.

## Deliverable

The `turtlebot_online_path_planning` package must be completed and compressed into a zip file. Do not forget to include in the README.md file:

* The name of the people in the group
* How to run your code or if some library must be installed
* If there is something special to explain.
* If there is anything that is not working properly.
* Several images or links to videos of the work done. IMPORTANT! Images must be embedded in the README.md (e.g., ```![](images/image.png)```).


# Optional Part: Open Motion Planning Library (OMPL)

Optionally, you can try to use the popular Open Motion Planning Library (OMPL) to implement the path planning algorithm. The first you have to do is to install this library. Unfortunately, python bindings for OMPL have to be installed by source and they use to present different problems in each version! 

To do it, follow the instructions in the [OMPL webpage](https://ompl.kavrakilab.org/installation.html). 

> WARNING: Version 1.6.0 and later do not work very well with Ubuntu 20.04.
> WARNING2: Since November 2023 OMPL presented some problems with Python.

Once the OMPL is installed in your PC, you should be able to run the notebook included in this package: [ompl_example.ipynb](notebooks/ompl_example.ipynb). This notebook will show you how to use the OMPL library to implement a basic path planning algorithm. Run it, understand the code, and complete whatever it is requested.
