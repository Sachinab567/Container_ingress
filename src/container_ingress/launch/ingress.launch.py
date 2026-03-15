import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_vegam_description = get_package_share_directory('Vegam_description')
    pkg_container_ingress = get_package_share_directory('container_ingress')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_container_ingress, 'worlds', 'container.world')

    # Start Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path, 'pause': 'false'}.items()
    )

    # Robot State Publisher & URDF processing
    vegam_urdf_path = os.path.join(pkg_vegam_description, 'urdf', 'Vegam.xacro')
    
    # We use xacro to process the file
    import xacro
    robot_description_config = xacro.process_file(vegam_urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Spawn the robot at the entrance of the container
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'Vegam', '-topic', 'robot_description', '-x', '-3.0', '-y', '0.0', '-z', '0.1', '-Y', '1.5708'],
        output='screen'
    )

    # Node for container ingress controller
    ingress_controller_node = Node(
        package='container_ingress',
        executable='ingress_controller',
        name='ingress_controller',
        output='screen'
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(ingress_controller_node)

    return ld
