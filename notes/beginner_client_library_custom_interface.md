# ROS2开发者笔记——自定义interface

## 消息

### 常用消息

#### std_msgs/Header

- 消息内容

```shell
""" For dashing """
$ ros2 msg show std_msgs/msg/Header 
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.
# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp
# Transform frame with which this data is associated.
string frame_id
""" For Foxy """
$ ros2 interface show std_msgs/msg/Header 
```

- 赋值

```python
class DemoNode(Node):
    def __init__(self):
        super().__init__('DemoNode')
        # demo_msg是用户定义的消息，包含了std_msgs/Header header成员
    	demo_msg.header.stamp = self.get_clock().now().to_msg()
```



### 自定义消息

#### 创建package

自定义的消息需要放到package中，创建package：

```shell
$ ros2 pkg create --build-type ament_cmake alex_msg
```

#### 添加消息

定义一个`CmdSetGimbalPose.msg`如下：

```python
uint8 UNIT_DEG=0
uint8 UNIT_RAD=1
uint8 FLAGS_RATE=0
uint8 FLAGS_ANGLE=10

std_msgs/Header header
uint8 unit
uint8 flags
float64 roll
float64 pitch
float64 yaw
```

自定义的消息内容有比较严格的命名规范，如果消息成员命名包含大写字母，会被当做常量。可以直接通过类名+常量名使用定义的常量：

```python
def callback(self, data):
	data.flags == CmdSetGimbalPose.FLAGS_ANGLE
```



