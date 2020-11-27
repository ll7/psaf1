# Start User Project

``` bash
# setup project
git clone https://github.com/ll7/psaf1.git
ln -s ~/psaf1/psaf_ros/ ~/carla-ros-bridge/catkin_ws/src/

# build project
cd ~/carla-ros-bridge/catkin_ws
catkin_make
source ~/.bashrc


# start CARLA + rviz + roscore
roscore
~/carla0.9.10.1/CarlaUE4.sh -windowed -ResX=160 -ResY=120
roslaunch carla_ros_bridge carla_ros_bridge_with_rviz.launch
# spawn vehicle
roslaunch carla_ego_vehicle carla_example_ego_vehicle.launch

# start test_script
rosrun psaf_steering hello.py
```
