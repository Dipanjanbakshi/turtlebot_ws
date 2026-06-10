#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            parameters=[
                {
                    'width': 640,
                    'height': 480,
                    'format': 'YUYV',
                }
            ],
            output='screen'
        ),
    ])