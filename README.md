# waypoint_planner
Local planner for TurtleBot3 that navigates through specified waypoints to a goal position.

## Installation
Change directory to your ROS workspace and type:

```bash
cd src
git clone https://github.com/chri633x/waypoint_planner.git
cd -

catkin_make waypoint_planner
source ~/.bashrc
```
## Run
Add the following command to your .bashrc file:

```bash
export TURTLEBOT3_MODEL=burger
```
Now run the following commands in different terminals:

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_bringup turtlebot3_remote.launch
rosrun map_server map_server "mapfile".yaml
roslaunch turtlebot3_navigation amcl.launch
```

Open Rviz, load "map" and "Robotmodel" and use "Initial pose estimate" to locate the robot. Open new terminal and run:

```bash
rosrun waypoint_planner waypoint_planner
```

Now you can build your path in Rviz by specifying waypoints with "Publish point" and eventually set a goal position with "2D Nav goal".
