from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    urdf_model_path = os.path.join(
        get_package_share_directory('geicar_description'),
        'src',
        'description',
        'geicar_description.urdf'
    ) 
    
    car_control_node_iyed = Node(
        package="car_control",
        executable="car_control_node_iyed",
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', str(urdf_model_path)])}],
    )
    
    joint_state_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
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

    ld.add_action(vehicle_controller_node)
    ld.add_action(car_control_node_iyed)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_node)


    return ld