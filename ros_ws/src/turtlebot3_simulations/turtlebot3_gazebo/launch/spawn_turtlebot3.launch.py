# Questo launch file si occupa di spawnare il singolo turtlebot 

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    bridge_params = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        model_folder+'_bridge.yaml'
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    namespace = LaunchConfiguration('namespace', default="t0")
    urdf = LaunchConfiguration('urdf', default=urdf_path)

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify x_pose of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify y_pose of the robot')
    
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='t0',
        description='Specify namespace of the robot')
    
    declare_urdf = DeclareLaunchArgument(
        'urdf', default_value=urdf_path,
        description='path to urdf file of the robot'
    )
    

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=[
            '-name', namespace,
            '-file', urdf,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'],
        output='screen',
    )
    

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_namespace)
    ld.add_action(declare_urdf)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)

    return ld
