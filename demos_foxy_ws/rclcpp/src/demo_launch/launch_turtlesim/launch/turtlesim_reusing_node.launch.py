from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
   turtle1 = Node(
      package='turtlesim',
      executable='turtlesim_node',
      name='turtle1'
   )
   turtle2 = Node(
      package='turtlesim',
      executable='turtlesim_node',
      name='turtle2'
   )
   return LaunchDescription([
      turtle1,
      turtle2
   ])