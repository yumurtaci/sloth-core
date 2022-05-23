# sloth-core

This code can fly!

## Methods and repositories
- [Mavros](http://wiki.ros.org/mavros)
- [PX4](https://github.com/PX4/PX4-Autopilot)
- [Geometric tracking controller](https://github.com/yumurtaci/mavros_controllers/tree/dev-sloth)
- [Minimum snap trajectory generation](https://github.com/ethz-asl/mav_trajectory_generation)
- State machine compatible with PX4 and Mavros
- Planner node to trigger trajectroy generation 

## Getting started

Create the workspace and initialize catkin
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --merge-devel
``` 

Clone the repository
``` bash
cd ~/catkin_ws/src
wstool init
git clone git@gitlab.lrz.de:sloth/sloth-core.git
``` 

Install the dependencies 
``` bash
cd ~/catkin_ws
wstool merge -t src src/sloth-core/dependencies.rosinstall
wstool update -t src -j4
``` 

Build the code with dependencies
``` bash
cd ~/catkin_ws
catkin b
``` 
> See [wiki](https://gitlab.lrz.de/groups/sloth/-/wikis/NVIDIA-Jetson-TX2-Setup#build-memory-issue) for **c++: internal compiler error: Segmentation fault (program cc1plus)** 

Terminal 1-2-3 etc.**change**
``` bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch **change**
``` 

Add starting commands for flight and px4 sitl gazebo
