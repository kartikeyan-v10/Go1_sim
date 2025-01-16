'''
import rclpy
from rclpy.node import Node
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        # Launch the navigate.py script (as an external process)
        ExecuteProcess(
            cmd=['python3', '/home/kartikeyan-v10/go1_ws/src/go1_navigation/go1_navigation/navigate.py'],
            output='screen'
        ),
        
        # Launch the image_capture node
        Node(
            package='go1_navigation',
            executable='image_capture',
            name='image_capture',
            
        ),
        
        # Launch rqt_image_view (ROS 2 GUI tool to view images)
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
            parameters=[],
        ),
    ])
'''
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the navigate.py script as an external process
        ExecuteProcess(
            cmd=['python3', '/home/kartikeyan-v10/go1_ws/src/go1_sim/go1_navigation/go1_navigation/navigate.py'],
            output='screen',
        ),
        
        # Launch the image_capture node from the go1_navigation package
        Node(
            package='go1_navigation',  # Package containing the image_capture node
            executable='image_capture',  # Executable for the image capture node
            name='image_capture',  # Node name
            output='screen',  # Output to the screen
        ),
        
        # Launch rqt_image_view node to visualize the image
        Node(
            package='rqt_image_view',  # Package for rqt_image_view
            executable='rqt_image_view',  # Executable for rqt_image_view
            name='rqt_image_view',  # Node name
            output='screen',  # Output to the screen
            parameters=[],  # Parameters if any are needed
        ),
    ])
