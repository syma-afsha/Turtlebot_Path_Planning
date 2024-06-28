
# Turtlebot On-line path planner

**This project belongs to the Universitat de Girona. It is forbidden to publish this project or any derivative work in any public repository.**

## How to run it:
<span style="color:red">Parameters of the task are tuned for "waffle" bot</span>

To execute the project file, at first it needs to be copied to the catkin workspace src floder and open a command prompt and run the following commands:
```bash
catkin build
```
after the it is successfully complied it needs to run a launch file. It can be done using the following commands
```bash
roslaunch turtlebot_online_path_planning gazebo.launch
```
This command will launch the gazebo, octomap server and rviz. After that, a new terminal needs to be opened and run the following commands. 
```bash 
rosrun turtlebot_online_path_planning turtlebot_online_path_planning_node.py
```
To begin the robot's path planning and movement, first activate the node. Following that, choose the 2D Nav Goal button from the toolbar. Afterward, click on a preferred spot on the map to set it as the target destination.

## Problems found:
1. If the goal position is set in an invalid position, the robot will not be able to reach the goal. 
2. As in the controller design, reverse functionality is added to save time while positioning the orientation according to path planning. But the trade off is sometimes it gets litte away from the planned path. 

## Example of use
The images below show the planner finding a route in an unknown area where the destination is set. In the video demostration, we have tired to find a critical route to goal for the robot and its navigation sucessfully. The youtube link is (https://youtu.be/aAkRD6W3dAM?si=lZnX5viLX9D7CfiH)
<div style="text-align: center">
    <img src='https://github.com/syma-afsha/Turtlebot_Path_Planning/blob/main/turtlebot_online_path_planning/imgs/test.png' width='600'/>
</div>
<div style="text-align: center">
    <img src='https://github.com/syma-afsha/Turtlebot_Path_Planning/blob/main/turtlebot_online_path_planning/imgs/test2.png' width='600'/>
</div>


## Discussions
In the implementation with RRT star a special functionly added as update_descendant_costs to compute the path which takes into account the deseendants nodes. This allows the algorithm to dynamically optimize the paths overtime ensuring that the decision about nodes connnections are made with the most current information leading to the discovery of the most cost efficient path to the goal. Without this funtion this can result in the selection of suboptimal paths and inability to fully exploit more efficient routes discovered later in the algorithms execution. Consiquently the path found to  the goal while valid may not be the  most efficient possible.In the controller when the robots moves to fix the orientation the linar velocity goes to zero. As a result of that, it takes time to fix the orientation. That's why reserve movement of robot is given to adjust orientation for reverse; the goal is essentially "behind" the robot. Additional functionality can also be added to the project to solve this small probems that we have found during testing.

## Conclusion
The purpose of this lab task was to navigate a virtual robot, from its starting point to a pre-set destination, avoiding any nearby obstacles along the way. As it was the first ros lab, we were able to learn new features of ros. 



