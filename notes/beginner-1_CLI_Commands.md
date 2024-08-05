# ROS2开发者笔记——命令大全

ROS2的命令以`ros2`开始

## ros2 pkg

### 创建功能包

```shell
ros2 pkg create --build-type ament_python demos_topic --dependencies rclpy
```



### 查看功能包路径

命令格式：`ros2 pkg prefix --share <pkg_name>`

例如：

```shell
$ ros2 pkg prefix --share turtlesim
```

可以进行命令组合实现更复杂的命令输入：

```shell
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz
```

## ros2 run

`ros2 run <package_name> <executable_name> [argv]`

其中argv主要包括：

- __node: 设置节点名字
- __ns: 设置namespace
- __params: 加载.yaml文件并传入参数
- --ros-args: 设置单个参数
- --remap/-r: 重映射

### 示例

我们首先看下小海龟节点默认启动后的情况：

```shell
$ ros2 run turtlesim turtlesim_node       # 启动小海龟
$ ros2 topic list                         # 查看话题
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
$ ros2 node list                          # 查看节点
/turtlesim
```



#### 设置节点命名空间

```shell
$ ros2 run turtlesim turtlesim_node --ros-args --remap __ns:=/demo --remap __node:=my_turtle

$ ros2 topic list                         # 查看话题
/demo/turtle1/cmd_vel
/demo/turtle1/color_sensor
/demo/turtle1/pose
$ ros2 node list                          # 查看节点
/demo/my_turtle
```

> 上面命令行不添加`--ros-args --remap`也能运行，只是会报警告，如下：
>
> [WARN] [1671161378.198301720] [rcl]: Found remap rule '__ns:=/demo'. This syntax is deprecated. Use '--ros-args --remap __ns:=/demo' instead.

#### 重映射话题名

```shell
$ ros2 run turtlesim turtlesim_node --ros-args --remap turtle1/cmd_vel:=my_cmd_vel   
$ ros2 run some_package some_ros_executable --ros-args -r turtle1/cmd_vel:=my_cmd_vel    #等效于上面这个

$ ros2 topic list                         # 查看话题
/my_cmd_vel
/turtle1/color_sensor
/turtle1/pose
```



#### 启动节点并设置参数

```shell
$ ros2 run demo_nodes_cpp talker __params:=demo_params.yaml
```

参考：http://docs.ros.org/en/foxy/How-To-Guides/Node-arguments.html

#### 直接设定参数

命令格式为：

```shell
$ ros2 run package_name executable_name --ros-args -p <param_name>:=<param_value>
```

例如：

```shell
ros2 run demo_nodes_cpp parameter_blackboard --ros-args -p some_int:=42 -p "a_string:=Hello world" -p "some_lists.some_integers:=[1, 2, 3, 4]" -p "some_lists.some_doubles:=[3.14, 2.718]"
```

查看参数：

```shell
$ ros2 param list
/parameter_blackboard:
  a_string
  some_int
  some_lists.some_doubles
  some_lists.some_integers
  use_sim_time
```

获取参数：

```shell
$ ros2 param get /parameter_blackboard a_string
String value is: Hello world
```



## ros2 interface

### 显示package中的所有消息

- ros2 interface package [pkg-name]

```shell
$ ros2 interface package nav_msgs
nav_msgs/msg/OccupancyGrid
nav_msgs/msg/MapMetaData
nav_msgs/msg/Path
nav_msgs/srv/GetMap
nav_msgs/msg/GridCells
nav_msgs/srv/SetMap
nav_msgs/msg/Odometry
nav_msgs/srv/GetPlan
```

### 查看话题的消息类型

- ros2 interface show [msg-type]



## ros2 topic

### 命令格式

--include-hidden-topics  hz
bw                       info
delay                    list
echo                     pub
find                     type

### 查看当前话题

- ros2 topic list

- ros2 topic list -t



### 话题发布命令

**命令格式**

ros2 topic pub [话题名] [消息类型] [消息赋值]

如果话题名不存在，则将新建，如果存在则向该话题发布。



**消息赋值**

通过双引号对消息进行赋值。

消息描述文件中定义了很多字段，

```shell
$ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
```

- 字符串类型字段

发布的消息类型包括字符串时，字符串需要通过单引号传递进来。

```shell
$ ros2 topic pub -r 1 /topic std_msgs/msg/String "{data: 'hello'}"
```

- 数组类型字段

如果一个消息`RobotStatus`定义如下：

```shell
float32[3] position
float32[3] velocity
```

那么对数组的赋值应该使用`[]`，例如：

```shell
$ ros2 topic pub my_msgs/msg/RobotStatus "{position: [1.0, 2.0, 0.0]}, velocity: [0.0, 0.0, 0.0]"
```



## service相关命令

--include-hidden-services  list 
call                       type
find

### 调用服务

`ros2 service call <service_name> <service_type> <arguments>`

```shell
# 清除turtle移动轨迹
$ ros2 service call /clear std_srvs/srv/Empty
# 生成一个新的小海龟
$ ros2 service call /spawn turtlesim/srv/Spawn "{x: 3.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```



## rqt

### rqt_gui

```shell
$ ros2 run rqt_gui rqt_gui
```

### 图像

发布图像

```shell
$ ros2 run rqt_image_view image_publisher
```

显示图像消息

```shell
$ ros2 run rqt_image_view rqt_image_view
```





