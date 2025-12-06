import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition # Use IfCondition for "enable" flags

def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('eval_ecosense')

    # --- Declare Nodes ---
    apriltag_node_left = Node(
        package='apriltag_ros',
        executable='apriltag_node_left',
        name='apriltag_node_left',
        output='screen',
        parameters=[
            {os.path.join(pkg_share, 'config/apriltag_params.yaml')},
            {'use_sim_time': LaunchConfiguration('use_sim_time', default='false')},
        ],
        remappings=[
            ('image_rect', '/usb_cam_left/image_raw'),
            ('camera_info', '/usb_cam_left/camera_info')
        ],
    )
    apriltag_node_right = Node(
        package='apriltag_ros',
        executable='apriltag_node_right',
        name='apriltag_node_right',
        output='screen',
        parameters=[
            {os.path.join(pkg_share, 'config/apriltag_params.yaml')},
            {'use_sim_time': LaunchConfiguration('use_sim_time', default='false')},
        ],
        remappings=[
            ('image_rect', '/usb_cam_right/image_raw'),
            ('camera_info', '/usb_cam_right/camera_info')
        ],
    )

    localization_eval_node = Node(
        package='eval_ecosense',
        executable='eval_localization',
        name='eval_localization',
        output='screen',
    )  

    ld.add_action(apriltag_node_left)
    ld.add_action(apriltag_node_right)
    ld.add_action(localization_eval_node)

    return ld