# Tracking
GERM team's Tracking robot 

 How to use:

$ cd ~/catkin_ws/src

$ git clone https://github.com/GERM-UDESC/Tracking.git

$ cd ~/catkin_ws/src/Tracking/sagan_plugin/build

$ export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/catkin_ws/src/Tracking/sagan_plugin/build

$ gazebo --verbose ../sagan.world

$ roscore

$ rostopic pub -- /sagan/vel_cmd geometry_msgs/Quaternion -50 -50 -50 -50

