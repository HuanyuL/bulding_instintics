# bulding_instintics
This is the turtlebot NUC setup
## ROS install
Install ROS-melodic by following this page [ROS installation](http://wiki.ros.org/melodic/Installation/Ubuntu)
## turtle_ws
Create turtle_ws and clone this repository to ``src``  
```
git clone https://github.com/HuanyuL/bulding_instintics .
```  
Install all the dependencies for packages
```
rosdep install --from-paths src --ignore-src -r -y
catkin build
```
## Important information  
usb-pc connection cable has to connect to fast charge socket
