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
git clone https://github.com/yumurtaci/sloth-core
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
> See [wiki/build memory issue](https://github.com/yumurtaci/sloth-core/wiki/NVIDIA-Jetson-TX2-Setup#build-memory-issue) for **c++: internal compiler error: Segmentation fault (program cc1plus)**


## Citation
In case you use this work as an academic context, please cite as the following.

```
@misc{yumurtaci_batuhan_sloth,
  author       = {Batuhan Yumurtaci},
  title        = {{Vision-Aided Learning Based Perching for a Bioinspired Metamorphic Multirotor}},
  howpublished = {\url{https://github.com/yumurtaci/sloth-core}},
  month        = oct,
  year         = 2022,
  }
```



