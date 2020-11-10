### Description

This package allows you to start a node for receiving data 
from the AMG88xx GridEYE 8x8 IR camera thermal sensor.
 When a heat source with a temperature higher than 
 threshold is detected, the detection node sends info message with 
 custom message format HeatAlert to /heat_sensor_output topic.
 After that, the node continues work, but for the first 10 seconds it ignores all sources of heat.


### Package installation on RPI

Install the package on RaspberryPi in the "standard" way:

```
cd ~/ros_catkin_ws/src
git clone https://github.com/voltbro/turtlebro_overheat_patrol_controller.git
cd ~/ros_catkin_ws
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic --pkg=turtlebro_overheat_patrol_controller
```

### Launch 

Launch only detector node:
```
roslaunch turtlebro_overheat_patrol_controller heat_patrol.launch
```