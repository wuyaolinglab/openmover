#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os

def generate_launch_description():

    driver_dir_1 = os.path.join(get_package_share_directory('lslidar_driver_n10p'), 'params', 'lsx10_1.yaml')
    driver_dir_2 = os.path.join(get_package_share_directory('lslidar_driver_n10p'), 'params', 'lsx10_2.yaml')
                     
    driver_node_1 = LifecycleNode(package='lslidar_driver_n10p',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',		#设置激光数据topic名称
                                output='screen',
                                emulate_tty=True,
                                namespace='lidar_1',
                                parameters=[driver_dir_1],
                                )

    driver_node_2 = LifecycleNode(package='lslidar_driver_n10p',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',		#设置激光数据topic名称
                                output='screen',
                                emulate_tty=True,
                                namespace='lidar_2',
                                parameters=[driver_dir_2],
                                )

    rviz_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'rviz', 'lslidar.rviz')

    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        output='screen')

    return LaunchDescription([
        driver_node_1,
        driver_node_2,
        rviz_node,
    ])

