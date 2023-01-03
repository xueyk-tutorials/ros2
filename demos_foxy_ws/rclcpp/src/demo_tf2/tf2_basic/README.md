
## 发布静态
启动节点
```shell
$ ros2 run tf2_basic static_turtle_tf2_broadcaster mystaticturtle 0 0 0 0.5 0 0
```
以上命令会广播一个mystaticturtle坐标系相对于world坐标系的坐标转换。

查看静态坐标系话题（先运行话题打印，再运行节点！）
```shell
$ ros2 topic echo /tf_static
transforms:
- header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: world
  child_frame_id: my_turtle
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.24740395925452294
      y: 0.0
      z: 0.0
      w: 0.9689124217106447
```

## 发布动态
启动
```shell
$ ros2 launch tf2_basic turtle_tf2_demo.launch.py
```
控制小海龟运动
```shell
$ ros2 run turtlesim turtle_teleop_key
```

查看tf广播
```shell
$ ros2 run tf2_ros tf2_echo world turtle1
# 或者
$ ros2 topic echo /tf
```
rviz2
打开rviz2
```shell
$ ros2 run rviz2 rviz2
```
Fixed Frame选择world，添加一个TF消息进行显示。通过控制小海龟，发现turtle1坐标系在运行。

## 两个小海龟
启动两个小海龟
```shell
$ ros2 run turtlesim turtlesim_node
$ ros2 service call /spawn turtlesim/srv/Spawn "{x: 3.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

启动tf
```shell
$ ros2 run tf2_basic turtle_tf2_broadcaster --ros-args -p "turtlename:=turtle1"
$ ros2 run tf2_basic turtle_tf2_broadcaster --ros-args -p "turtlename:=turtle2"
```

查看两个小海龟之间的转换关系
这里是turtle2在turtle1坐标系下的平移与旋转。
```shell
$ ros2 run tf2_ros tf2_echo turtle1 turtle2
```