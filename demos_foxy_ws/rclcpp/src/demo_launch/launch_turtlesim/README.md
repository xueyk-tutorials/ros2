# launch_turtlesim

Ref:[https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#id8](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#id8)

## turtlesim_mimic
### 启动launch
```shell
$ ros2 launch launch_turtlesim turtlesim_mimic_launch.py
```
### 发布控制指令
启动小海龟
```shell
$ ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

## turtlesim_parameter
### 在launch文件中设置参数
```shell
ros2 launch launch_turtlesim turtlesim_world_1.launch.py
```

### 从.yaml文件中加载参数
如果参数数量非常多，比较好的方式是将这些参数统一放到**.yaml**文件中管理。
```shell
ros2 launch launch_turtlesim turtlesim_world_2.launch.py
```

### 查看参数
```shell
$ ros2 param get /turtlesim2/sim background_r
```

## 命名空间
为了避免启动多个同名的节点时，topic发生冲突，可以将同名节点放到不同的命名空间中。最简单的方式是在launch文件中使用`namespace`指定节点启动的命名空间。但如果launch中节点数量非常多时，每个节点都需要指定命名空间，写起来很繁琐。
ROS2提供了`PushRosNamespace`，可以给launch文件中所有节点（包括嵌套的节点）一个全局的命名空间。


## 节点命名
如果需要一个节点重复启动多次时，为了避免造成冲突，在生成节点时可以通过`name`对各节点进行不同命名。
```shell
$ ros2 launch launch_turtlesim turtlesim_parameter_overrides.launch.py
```

```shell
$ ros2 node list
/turtle1
/turtle2
```
这样就可以对不同名称的节点设置参数了：
```shell
$ ros2 param set /turtle1 background_r 0
```
## 参数覆盖
当我们在外层launch文件去include某个launch文件，可以通过'launch_arguments'进行参数覆盖，将外层launch文件定义的参数去覆盖被include的launch文件默认参数。

- 启动launch
```shell
$ ros2 launch launch_turtlesim turtlesim_parameter_overrides.launch.py
```
- 查看参数
```shell
$ ros2 param get /sim background_r
```

## 设置配置文件
例如rviz启动时，可以传入.rviz配置文件。

```shell
$ ros2 launch launch_turtlesim turtlesim_rviz.launch.py
```