"""
static_transform_publisher是tf2_ros包自带的静态坐标系广播节点！
这里演示通过launch启动该节点
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '1', '0', '0', '0', 'world', 'mystaticturtle']
      ),
   ])