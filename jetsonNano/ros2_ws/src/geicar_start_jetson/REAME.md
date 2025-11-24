# geicar_start_jetson — Quick start

## Prerequisites
- Build the workspace:
    ```
    colcon build --symlink-install
    ```
- Source ROS 2 and the workspace:
    ```
    source install/setup.bash
    ```

## Start the full launch
Run the main launch file:
```
ros2 launch geicar_start_jetson geicar.jetson.launch.py
```

## Start with some package/node disabled
The launch file can accept boolean launch arguments to enable/disable sub-systems. Example to disable camera and lidar:
```
ros2 launch geicar_start_jetson geicar.jetson.launch.py disable_camera:=true disable_lidar:=true
```

## Auto start on boot

Remerber that on the Jetson the systemd service already exists to auto start the launch file on boot. Edit the start script to disable or enable some packages if needed at `/usr/local/bin/geicar_start_jetson.sh`.