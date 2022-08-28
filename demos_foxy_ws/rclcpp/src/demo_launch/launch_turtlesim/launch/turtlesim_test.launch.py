import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 加载参数、加载配置文件：通过get_package_share_directory获取package的路径，然后便可以获取其package下所有文件路径了
    config_yaml = os.path.join(
      get_package_share_directory('turtlesim'),
      'config',
      'turtlesim.yaml'
      )
    print(">>>config_yaml=", config_yaml)

    # 定义参数
    background_r_launch_arg = DeclareLaunchArgument(
      'background_r', default_value=TextSubstitution(text='0')
    )
    print(">>>background_r_launch_arg=", background_r_launch_arg)

    # 先定义node
    turtle1 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtle1',
        parameters=[{
            'background_r': LaunchConfiguration('background_r')    # 使用参数
         }]
    )

    # include其他launch文件，并传入参数
    launch_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('launch_turtlesim'), 'launch'),
                                      '/turtlesim_world_1.launch.py']),
        launch_arguments={'background_r': TextSubstitution(text='250')}.items(),
        )

    return LaunchDescription([
        turtle1,                                                   # 启动node
        launch_nodes
    ])