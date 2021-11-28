# turtlebot_walker
This assignment is the Week 13 assignment for ENPM808X. This involves crreating a simple walker algorithm for turtlebot3.

## Dependencies
- ROS Melodic
- Ubuntu 18.04
- Turtlebot 3 Packages and Simulation Packages(Use this manual linked [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/))
- C++ 11

## Steps to run the launch file
### Building the package
```
cd <location_of_catkin_ws>/src
git clone --recursive https://github.com/PrannoyNamala/turtlebot_walker
cd .. && catkin_make
```

## Running the walker node
```
# Launch without recording
roslaunch walker walker.launch
# Record a rosbag for 15 seconds
roslaunch walker walker.launch record:=true duration:=15
```
