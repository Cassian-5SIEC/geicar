import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition # Use IfCondition for "enable" flags

def generate_launch_description():
    ld = LaunchDescription()

    # --- Declare Launch Arguments ---
    # We create a "disable" argument for each node. 
    # By default, they are 'false' (meaning the node IS launched).
    # Setting disable_camera:=true will skip launching the camera.

    declare_disable_lidar_arg = DeclareLaunchArgument(
        'disable_lidar',
        default_value='false',
        description='Disable the RPLidar node'
    )

    declare_disable_camera_arg = DeclareLaunchArgument(
        'disable_camera',
        default_value='false',
        description='Disable the USB Camera node'
    )

    declare_disable_system_check_arg = DeclareLaunchArgument(
        'disable_system_check',
        default_value='false',
        description='Disable the system check node'
    )

    declare_disable_bridge_arg = DeclareLaunchArgument(
        'disable_bridge',
        default_value='false',
        description='Disable the bridge node'
    )

    # --- Define Nodes with Conditions ---

    # We add a 'condition' to each node.
    # UnlessCondition(LaunchConfiguration('disable_camera')) means:
    # "Launch this node UNLESS the 'disable_camera' argument is 'true'."

    lidar_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('disable_lidar'))
    )

    usb_cam_share = get_package_share_directory('usb_cam')

    usb_cam_share = get_package_share_directory('usb_cam')

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace="usb_cam_0",
        parameters=[os.path.join(usb_cam_share, 'config', 'params_1.yaml')],
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('disable_camera'))
    )

    camera_node_2 = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace="usb_cam_1",
        parameters=[os.path.join(usb_cam_share, 'config', 'params_2.yaml')],
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('disable_camera'))
    )


    system_check_ack_node = Node(
        package="system_check_ack",
        executable="system_check_ack_node",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('disable_system_check'))
    )

    bridge_node = Node(
        package="network_hmi",
        executable="bridge_node",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('disable_bridge'))
    )

    # --- Add Actions to Launch Description ---

    # Add the argument declarations
    ld.add_action(declare_disable_lidar_arg)
    ld.add_action(declare_disable_camera_arg)
    ld.add_action(declare_disable_system_check_arg)
    ld.add_action(declare_disable_bridge_arg)

    # Add the nodes (they will only be executed if their condition is met)
    ld.add_action(lidar_node)
    ld.add_action(camera_node)
    ld.add_action(camera_node_2)
    ld.add_action(system_check_ack_node)
    ld.add_action(bridge_node)

    return ld