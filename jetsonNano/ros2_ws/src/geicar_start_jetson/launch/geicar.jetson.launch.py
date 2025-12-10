import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import UnlessCondition # Use IfCondition for "enable" flags
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('geicar_start_jetson')

    urdf_model_path = os.path.join(
        get_package_share_directory('ecosense_description'),
        'src',
        'description',
        'ecosense_description.urdf'
    )

    # --- Declare Launch Arguments ---
    # We create a "disable" argument for each node. 
    # By default, they are 'false' (meaning the node IS launched).
    # Setting disable_camera:=true will skip launching the camera.
    disable_robot_state_publisher_arg = DeclareLaunchArgument(
        'disable_robot_state_publisher',
        default_value='false',
        description='Disable the robot state publisher node'
    )
    
    declare_disable_lidar_arg = DeclareLaunchArgument(
        'disable_lidar',
        default_value='false',
        description='Disable the RPLidar node'
    )

    declare_disable_lio_arg = DeclareLaunchArgument(
        'disable_lio',
        default_value='false',
        description='Disable the Laser Odometry node'
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

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (Gazebo/Bag) if true'
    )

    # --- Define Nodes with Conditions ---

    # We add a 'condition' to each node.
    # UnlessCondition(LaunchConfiguration('disable_camera')) means:
    # "Launch this node UNLESS the 'disable_camera' argument is 'true'."

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', str(urdf_model_path)])}, 
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=UnlessCondition(LaunchConfiguration('disable_robot_state_publisher'))
    )

    lidar_node = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyUSB0',
                         'serial_baudrate': 256000,
                         'frame_id': 'rplidar_link',
                         'inverted': False,
                         'angle_compensate': True,
                         'scan_mode': 'Sensitivity'},
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('disable_lidar'))
    )

    usb_cam_share = get_package_share_directory('usb_cam')

    camera_node_1 = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace="usb_cam_right",
        parameters=[{os.path.join(usb_cam_share, 'config', 'params_right.yaml')},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('disable_camera'))
    )

    camera_node_2 = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace="usb_cam_left",
        parameters=[{os.path.join(usb_cam_share, 'config', 'params_left.yaml')},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('disable_camera'))
    )


    system_check_ack_node = Node(
        package="system_check_ack",
        executable="system_check_ack_node",
        emulate_tty=True,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=UnlessCondition(LaunchConfiguration('disable_system_check'))
    )

    bridge_node = Node(
        package="network_hmi",
        executable="bridge_node",
        emulate_tty=True,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'image_topic': '/usb_cam_left/image_raw/compressed'}],
        condition=UnlessCondition(LaunchConfiguration('disable_bridge'))
    )

    rf2o_laser_odometry_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom',
            'publish_tf' : True,
            'base_frame_id' : 'base_link',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 15.0},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=UnlessCondition(LaunchConfiguration('disable_lio'))
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        remappings=[
            ('/odometry/filtered', '/odom'),     # Rename output odom
        ],
        parameters=[{os.path.join(pkg_share, 'config/ekf.yaml')},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    slam_toolbox_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_share, 'config', 'slam_params.yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
    )

    # --- Add Actions to Launch Description ---

    # Add the argument declarations
    ld.add_action(disable_robot_state_publisher_arg)
    ld.add_action(declare_disable_lidar_arg)
    ld.add_action(declare_disable_lio_arg)
    ld.add_action(declare_disable_camera_arg)
    ld.add_action(declare_disable_system_check_arg)
    ld.add_action(declare_disable_bridge_arg)
    ld.add_action(declare_use_sim_time_arg)

    # Add the nodes (they will only be executed if their condition is met)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(lidar_node)
    ld.add_action(camera_node_1)
    ld.add_action(camera_node_2)
    ld.add_action(system_check_ack_node)
    ld.add_action(bridge_node)
    ld.add_action(rf2o_laser_odometry_node)
    #ld.add_action(robot_localization_node)
    #ld.add_action(slam_toolbox_launch_file)

    return ld
