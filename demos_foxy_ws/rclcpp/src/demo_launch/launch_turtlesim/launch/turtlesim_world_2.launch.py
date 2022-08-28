import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('launch_turtlesim'),
      'config',
      'turtlesim.yaml'
   )
   print("------------------------------------------")
   print('config=', config)
   print("------------------------------------------")
   return LaunchDescription([
      Node(
         package='turtlesim',
         executable='turtlesim_node',
         # namespace='turtlesim2',
         output='screen',
         name='sim',
         parameters=[config]
      )
   ])
