# How to compile the ros2_ws on the Raspberry

To accelerate the compilation, the workspace will be compiled on the Jetson Orin Nano and then copied to the Raspberry Pi 4.

## On the Jetson Orin Nano
1. Make sure the workspace is up to date:
2. Navigate to the ros2_ws directory:
```sh
cd ~/geicar/raspberryPI3/ros2_ws
```
3. Install dependencies:
```sh
rosdep init # If not done already
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
4. Compile the workspace:
```sh
colcon build --symlink-install
```
5. Copy the compiled workspace to the Raspberry Pi 4:
```sh
rsync -avz --progress ~/geicar/raspberryPI3/ros2_ws <PI_Username>@<Raspberry_Pi_IP_Address>:~/ros2_ws
```
## On the Raspberry Pi 4
1. SSH into the Raspberry Pi 4:
```sh
ssh <PI_Username>@<Raspberry_Pi_IP_Address>
```
2. Source the ROS 2 and workspace setup files:
```sh
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
```
3. Launch the general launch file :
```sh
ros2 launch geicar_start geicar.launch.py
```

**Deprecated**
---

**The raspberry Pi does not have the resources to compile the whole workspace at the same time.** It is better to compile the packages one by one (some packages can take 20 minutes to compile..)\
**In any case, the first package to compile is the "interfaces" package** because the other packages depend on it.\
**The compilation must ALWAYS be launched at the workspace root (~/ros2_ws)**

Syntax of the compilation command (```colcon build```):
* ```colcon build``` to compile all the workspace
* ```colcon build --packages-select "NameOfThePackage"```

1. Compilation of the "interfaces" package
```sh
cd ~/ros2_ws
colcon build --packages-select interfaces
```

1. Then, you can compile the other packages:
```sh
cd ~/ros2_ws
colcon build --packages-select carla_msgs
colcon build --packages-select can
colcon build --packages-select joystick
colcon build --packages-select car_control
colcon build --packages-select imu_tools
colcon build --packages-select imu_filter_madgwick
colcon build --packages-select imu_complementary_filter
colcon build --packages-select simulation
colcon build --packages-select system_check
```

1. After the compilation, you have to source the ros environment:
```sh
source ~/ros2_ws/install/local_setup.bash
source /opt/ros/humble/setup.bash
```

