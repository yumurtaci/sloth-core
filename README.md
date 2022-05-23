# sloth-core

This code can fly!

## Getting started

Create the workspace and initialize catkin
```bash
mkdir -p ~/**change**/src
cd ~/**change**
catkin init
catkin config --merge-devel
``` 

Clone the repository
``` bash
cd ~/**change**/src
wstool init
git clone **change**
``` 

Install the dependencies 
``` bash
cd ~/**change**
wstool merge -t src src/**change**/dependencies.rosinstall
wstool update -t src -j4
``` 

Build the code with dependencies
``` bash
cd ~/**change**
catkin b **change**
``` 

Terminal 1 (px4_usb.launch is a custom launch file -> push)
``` bash
roslaunch mavros px4_usb.launch
``` 

Terminal 2
``` bash
cd ~/**change**
source devel/setup.bash
roslaunch **change**
``` 

Add starting commands for flight and px4 sitl gazebo
