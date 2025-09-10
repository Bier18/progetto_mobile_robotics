#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool, Hyungyu Kim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

INITIAL_POSE = {
    't0': ('-5.0', '0.0'),
    't1': ('-4.0', '0.0'),
    't2': ('-3.0', '0.0'),
    't3': ('-2.0', '0.0'),
    't4': ('-1.0', '0.0'),
    't5': ('0.0', '0.0'),
    't6': ('1.0', '0.0'),
    't7': ('2.0', '0.0'),
    't8': ('3.0', '0.0'),
    't9': ('4.0', '0.0'),
    't10': ('5.0', '0.0'),
    't11': ('6.0', '0.0'),
    't12': ('7.0', '0.0')
}

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    
    ld = LaunchDescription()
    
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))
    
    robot_list = []
    
    for i in range(13):
        namespace = f't{i}'
        x_pose, y_pose = INITIAL_POSE[namespace]
        sdf_file = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models',
            'turtlebot3_burger',
            f'model{i}.sdf'
        )
        urdf_path= os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'urdf',
            f'burger{i}.urdf'
        )

        robot_list.append(namespace)

        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
                'namespace': namespace,
                'urdf' : sdf_file,
            }.items()
        )

        with open(urdf_path, 'r') as infp:
            robot_desc = infp.read()

        robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        )

        ld.add_action(spawn_turtlebot_cmd)
        ld.add_action(robot_state_publisher_cmd)
    
    global_pose_publisher_cmd = Node(
        package='utilities',
        executable='global_pose_publisher',
        name='global_pose_publisher',
        output='screen',
        parameters=[{
            'robots': robot_list,
            'use_sim_time': True
        }]
    )

    odom_to_base_broadcatser_cmd = Node(
        package='utilities',
        executable='odom_to_base_broadcaster',
        name='odom_to_base_broadcaster',
        output='screen',
        parameters=[{
            'robots': robot_list,
            'use_sim_time': True
        }]
    )

    static_map_to_odom_broadcaster_cmd = Node(
        package='utilities',
        executable='static_map_to_odom_broadcaster',
        name='static_map_to_odom_broadcaster',
        output='screen',
        parameters=[{
            'robots': robot_list,
            'use_sim_time': True
        }]
    )

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(global_pose_publisher_cmd)
    ld.add_action(odom_to_base_broadcatser_cmd)
    ld.add_action(static_map_to_odom_broadcaster_cmd)

    return ld
