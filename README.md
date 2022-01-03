# Cyberdog rviz2 plugin

## Preview

![preview](readme_pics/preview.gif)

## dependencies 
Make sure motion_msgs from cyberdog repo is installed, and use cyclonedds for service call between machines
```
sudo apt install ros-foxy-rmw-cyclonedds-cpp
```

## to launch 
source your repos
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2
```
 Panels -> Add new panel -> mission panel