# Guide to compile the ros2_ws on the Jetson
For now, no specific instructions for the Jetson Nano. This guide will be updated later.

Deprecated
---
1. Go into the docker container ros-humble 
```sh
sudo docker start -ai ros-humble
```

2. Compile the workspace
```sh
cd ~/ros2_ws
colcon build
```

3. Source the installation
```sh
source install/local_setup.bash
```
