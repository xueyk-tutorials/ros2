import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
   turtlesim_world_2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_turtlesim'), 'launch'),
         '/turtlesim_world_2.launch.py'])
      )
   turtlesim_world_2_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('turtlesim2'),
         turtlesim_world_2,
      ]
   )
   return LaunchDescription([
      turtlesim_world_2_with_namespace
   ])