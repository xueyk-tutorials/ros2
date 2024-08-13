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
$ ros2 pkg create --build-type ament_cmake alex_msgs
```

#### 添加消息

ROS2中使用IDL（即[Interface Definition Language](https://www.omg.org/spec/IDL/)）技术定义消息。

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

命名的格式为：

```shell
^(?!.*__)(?!.*_$)[a-z][a-z0-9_]*$'
```

IDL类型包括如下

```shell
bool
byte
char
float32,float64
int8,uint8
int16,uint16
int32,uint32
int64,uint64
string
```

自定义的消息内容有比较严格的命名规范，如果消息成员命名包含大写字母，会被当做常量。可以直接通过类名+常量名使用定义的常量：

```python
def callback(self, data):
	data.flags == CmdSetGimbalPose.FLAGS_ANGLE
```



#### 构建

1. 修改CMakeLists.txt

   打开CMakeLists.txt后，需要以下工作：

   - 查找生成器库；
   - 添加需要生成的消息文件；
   - 导出。

```shell
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/CmdSetGimbalPose.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)
```

> 如果没有使用`rosidl_default_generators`，那么无法在终端命令行中使用该package。
>
> 通过`ros2 pkg list`是能够查看到alex_msgs这个package，但是无法使用这个package。
>
> 也就是通过命令`ros2 topic pub`无法正常发布该package的消息。

2. 修改package.xml

   由于需要构建消息，故需要rosidl_default_generators，由于运行时也需要使用消息，故需要rosidl_default_runtime。

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

### 在消息中添加数组

ROS2支持在消息内定义数组，在C++中进行处理时，“数组”被当作std::list类型。

例如定义一个消息Desk.msg如下：

```shell
std_msgs/Header header

uint8[] books    # 不限数量的书籍
uint8[<=2] cups  # 最多两个杯子
```



- 给数组赋值操作

使用assign()函数实现给消息数组成员赋值。

```c++
uint8_t my_books[10] = {1, 2, 3, 4};
uint8_t my_cups[2] = {};
    
Desk desk;
desk.assgin(my_books, my_books+sizeof(my_books));
```

- 拷贝

使用std::copy()将数据从消息数组成员拷贝出数据。

```c++
std::copy(desk.begin(), desk.end(), my_cups);
```



## 参考

https://design.ros2.org/articles/idl_interface_definition.html

