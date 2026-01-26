from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # urdf_model_path = os.path.join(
    #     get_package_share_directory('geicar_description'),
    #     'src',
    #     'description',
    #     'geicar_description.urdf'
    # )

    # --- Declare Launch Arguments ---

    # disable_robot_state_publisher_arg = DeclareLaunchArgument(
    #     'disable_robot_state_publisher',
    #     default_value='false',
    #     description='Disable the robot state publisher node'
    # )

    disable_car_control_arg = DeclareLaunchArgument(
        'disable_car_control',
        default_value='false',
        description='Disable the car control node'
    )

    disable_can_arg = DeclareLaunchArgument(
        'disable_can',
        default_value='false',
        description='Disable the CAN nodes'
    )

    disable_joystick_arg = DeclareLaunchArgument(
        'disable_joystick', 
        default_value='false',
        description='Disable the joystick nodes'
    )

    disable_system_check_arg = DeclareLaunchArgument(
        'disable_system_check',
        default_value='false',
        description='Disable the system check node'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (Gazebo/Bag) if true'
    )

    # --- Initialize Launch Description ---
    
    ld = LaunchDescription()

    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': Command(['xacro ', str(urdf_model_path)])}, 
    #                 {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #     condition=UnlessCondition(LaunchConfiguration('disable_robot_state_publisher'))
    # )

    joystick_node = Node(
        package="joystick",
        executable="joystick_ros2.py",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('disable_joystick'))
    )

    joystick_to_cmd_node = Node(
        package="joystick",
        executable="joystick_to_cmd",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('disable_joystick'))
    )

    can_rx_node = Node(
        package="can",
        executable="can_rx_node",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('disable_can'))
    )

    can_tx_node = Node(
        package="can",
        executable="can_tx_node",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('disable_can'))
    )

    car_control_node = Node(
        package="car_control",
        executable="car_control_node",
        emulate_tty=True,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=UnlessCondition(LaunchConfiguration('disable_car_control'))
    )


    # madgwick_config_dir = os.path.join(get_package_share_directory('imu_filter_madgwick'), 'config')

    # imu_filter_madgwick_node = Node(
    #     package="imu_filter_madgwick",
    #     executable="imu_filter_madgwick_node",
    #     parameters=[os.path.join(madgwick_config_dir, 'imu_filter.yaml')],
    #     emulate_tty=True
    # )

    imu_calibration_node = Node(
        package="imu_calibration",
        executable="imu_calibration_node",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        emulate_tty=True
    )


    system_check_node = Node(
        package="system_check",
        executable="system_check_node",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('disable_system_check'))
    )

    vehicle_controller_node = Node(package='car_control',
                                executable='vehicle_controller',
                                parameters=[ {
                                            'body_width': 0.985,          # meters
                                            'body_length': 0.523,         # meters
                                            'wheel_radius': 0.095,        # meters
                                            'wheel_width': 0.076,         # meters
                                            'max_steering_angle': 0.6108652,   # radians
                                            'max_velocity': 0.5          # m/s
                                        }],
                                output='screen')

    safety_node = Node(
        package="safety_node",
        executable="safety_node",
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Arguments Actions
    # ld.add_action(disable_robot_state_publisher_arg)
    ld.add_action(disable_car_control_arg)
    ld.add_action(disable_can_arg)
    ld.add_action(disable_joystick_arg)
    ld.add_action(disable_system_check_arg)
    ld.add_action(use_sim_time_arg)

    # Nodes Actions
    ld.add_action(joystick_node)
    ld.add_action(joystick_to_cmd_node)
    ld.add_action(can_rx_node)
    ld.add_action(can_tx_node)
    ld.add_action(car_control_node)
    # ld.add_action(imu_filter_madgwick_node)   # Not needed because not using Magnetometer
    ld.add_action(imu_calibration_node)
    ld.add_action(system_check_node)
    #ld.add_action(robot_state_publisher_node)
    ld.add_action(vehicle_controller_node)
    ld.add_action(safety_node)


    return ld
