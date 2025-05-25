from launch import LaunchDescription
from launch_ros.actions import Node

"""
This launch file starts the camera detection node and the custom robot controller node.
The camera detection node is responsible for processing images from the camera,
while the custom robot controller node handles the robot's movements and actions.
"""

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cameraDetection',
            executable='camera_node',
            name='camera_node'
        ),
        Node(
            package='CustomRobotController',
            executable='robot_controller_node',
            name='robot_controller_node'
        ),
    ])
