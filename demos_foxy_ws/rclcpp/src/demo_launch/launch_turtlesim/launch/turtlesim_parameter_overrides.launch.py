import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    parameter_overrides_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_turtlesim'), 'launch'),
            '/turtlesim_world_1.launch.py']),
        launch_arguments={'background_r': TextSubstitution(text='250')}.items(),
        )
    return LaunchDescription([
        parameter_overrides_nodes
    ])

