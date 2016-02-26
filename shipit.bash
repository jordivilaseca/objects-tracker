#!/bin/bash

# Deploy ROS package to server.
scp -r ~/catkin_ws/src/objects_tracker/{launch,include,src,CMakeLists.txt,package.xml} ros@192.168.2.245:~/tracker_ws/src/objects_tracker/
echo "source /opt/ros/indigo/setup.bash; source ~/tracker_ws/devel/setup.bash; cd ~/tracker_ws; catkin_make; exit" | ssh ros@192.168.2.245