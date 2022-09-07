# launch



## launch-python

一个典型的`launch.py`文件如下：

```python
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='flocking',
            node_namespace='',
            node_executable='leader',
            node_name='leader',
            output='screen'
        ),
        Node(
            package='flocking',
            node_namespace='flock',
            node_executable='follower',
            node_name='follower',
            output='screen'
        ),
    ])
```



- node_namespace

命名空间，节点内所有话题名（只包括不带根节点'/'的话题名）将归属到这个命名空间内。

- output

是否将调试信息打印输出至终端



launch文件可以是独立于ros2 package的，也可以是在package下的launch文件夹中的，二者的启动方式不同：

```shell
### 1. 执行当前目录下的launch文件 
$ ros2 launch <launch_file_name.py>

### 2. 执行package下的launch文件
$ ros2 launch <package_name> <launch_file_name.py>
```



