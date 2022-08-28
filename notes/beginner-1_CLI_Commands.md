# ROS2开发者笔记——命令大全

ROS2的命令以`ros2`开始

## ros2 pkg

ros2 pkg create --build-type ament_python demos_topic --dependencies rclpy

## ros2 run

`ros2 run <package_name> <executable_name> [argv]`

其中argv主要包括：

- __node: 设置节点名字
- __ns: 设置namespace
- __params: 加载.yaml文件并传入参数
- --ros-args: 设置单个参数

### 示例

#### 设置节点命名空间

```shell
$ ros2 run demo_nodes_cpp talker __ns:=/demo __node:=my_talker chatter:=my_topic
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

显示package中的所有消息

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



## topic相关命令

--include-hidden-topics  hz
bw                       info
delay                    list
echo                     pub
find                     type

### 查看当前话题

ros2 topic list

ros2 topic list -t

### 查看话题的消息类型

ros2 interface show <msg type>

### 话题发布命令

ros2 topic pub

```shell
$ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
```

发布的消息类型包括字符串时，字符串需要通过单引号传递进来。

```shell
$ ros2 topic pub -r 1 /topic std_msgs/msg/String "{data: 'hello'}"
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





