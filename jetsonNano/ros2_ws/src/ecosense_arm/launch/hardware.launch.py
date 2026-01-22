from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    # Add launch argument

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Visualize the robot in RViz'
    )

    robot_urdf_path_arg = DeclareLaunchArgument(
        'robot_urdf_path',
        default_value=PathJoinSubstitution(
            [FindPackageShare('ecosense_arm'), 'config', 'ecosense_arm.urdf.xacro']
        ),
        description='Path to the robot URDF file'
    )

    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'), ' ',
                LaunchConfiguration('robot_urdf_path'),
                ' ',
                'use_fake_hardware:=false'
            ]
        ),
        value_type=str
    )

    robot_description = {'robot_description': robot_description_content}

    # robot_state_pub_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[robot_description]
    # )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [FindPackageShare('ecosense_arm'), 'config', 'ros2_controllers.yaml']
            ),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after joint_state_broadcaster
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Delay loading and starting robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )


    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('so_100_arm'), 'config', 'urdf.rviz'])]
    )

    nodes = [
        # robot_state_pub_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        gripper_controller_spawner,
        # rviz_node,
    ]

    return LaunchDescription([rviz_arg, robot_urdf_path_arg] + nodes) 