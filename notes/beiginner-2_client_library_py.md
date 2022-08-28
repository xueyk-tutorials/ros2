# ROS2开发者笔记——client library

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



## 发布者和订阅者

不要在`wall_timer`的回调函数中直接读取摄像头！会造成该节点的消息订阅无法正常工作。

#### 创建功能包

```shell
$ ros2 pkg create --build-type ament_python py_pubsub
```

#### 创建发布者

在文件夹src/py_pubsub/py_pubsub下创建文件publisher_member_function.py：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
```

- 设置依赖项

打开功能包的package.xml文件，然后还需要添加依赖项，放到ament_python下边： 

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

 设置程序入口

 接下来打开setup.py文件，同样需要补充以下内容： 

```python
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',

```

  然后在 entry_points 下添加如下内容： 

```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],},

```

 检查setup.cfg文件

 setup.cfg文件中内容是自动添加的，可以打开看下，内容如下： 

```shell
[develop]script-dir=$base/lib/py_pubsub
[install]install-scripts=$base/lib/py_pubsub

```

#### 创建订阅者

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
 
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
```

由于该节点的依赖项和发布者一样，我们就不需要修改package.xml和 setup.cfg文件了，不过程序入口 **setup.py**还是得加一些内容： 

```shell
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],},
```

#### 编译

```shell
alex@alex-xiaomi:/mnt/d/WSL/ros2_learn/gyj_ws$ colcon build --packages
-select py_pubsub
Starting >>> py_pubsub
Finished <<< py_pubsub [1.66s]

Summary: 1 package finished [1.86s]
```

如果需要检查依赖是否齐全并安装所有依赖，可以运行

```shell
$ rosdep install -i --from-path src --rosdistro foxy -y
```

#### 运行测试

1、打开一个终端，设置工作空间环境变量，运行发布节点

```shell
$ . install/setup.bash
```

2、再打开一个终端，运行订阅节点

```shell
$ . install/setup.bash   
$ ros2 run py_pubsub listener
```

## action

之前介绍到的话题和服务是ROS中最重要的两种底层通信机制，但也并不是说能解决所有应用层的问题，举一个例子，如果要让机械臂抓取一个物体，我们不仅要发出指令，还需要获取机械臂的实时反馈，如果直接用话题和服务也可以实现，不过一下得上好几个，有点麻烦。   所以针对类似的场景，ROS推出了一个应用级的通信机制——动作（action），主要结局需要运行一段时间的机器人任务。

action也并不是一个全新的机制，而是由底层的三个话题和服务组成：

- 一个任务目标（Goal，服务），
- 一个执行结果（Result，服务），
- 周期数据反馈（Feedback，话题）。   

action是可抢占式的，由于需要执行一段时间，比如执行过程中你不想跑了，那可以随时发送取消指令，动作终止，如果执行过程中发送一个新的action目标，则会直接中断上一个目标开始执行最新的任务目标。   总体上来讲，action是一个客户端/服务器的通信模型，客户端发送一个任务目标，服务器端根据收到的目标执行并周期反馈状态，执行完成后反馈一个执行结果。  

高级应用：

http://design.ros2.org/articles/actions.html

#### 创建action

参考：http://docs.ros.org/en/rolling/Tutorials/Actions/Creating-an-Action.html

创建工作空间和一个package，命名为**action_tutorials_interfaces**：

```shell
$ mkdir -p action_ws/src
$ cd action_ws/src
$ ros2 pkg create action_tutorials_interfaces
```

在**package**下创建一个**action**文件夹，然后创建一个**Fibonacci.action**文件，注意action文件命名必须以大写字母开头。

由于action包含了两个服务和一个话题，action文件格式如下：

```python
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

我们创建的**Fibonacci.action**文件内容如下：

```python
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

编辑CMakeLists.txt文件

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

编辑package.xml

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

编译

```shell
$ colcon build
```

查看action

```shell
$ . install/setup.bash
# ros2 rolling
$ ros2 interface show action_tutorials/action/Fibonacci
# ros2 dashing
$ ros2 action show interfaces_pkg/action/Fibonacci
```

#### 取消动作

https://github.com/ros2/examples/tree/master/rclpy/actions

[Actions — rclpy 0.6.1 documentation (ros2.org)](https://docs.ros2.org/foxy/api/rclpy/api/actions.html)

#### 示例

服务端

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class Demo(Node):
    def __init__(self):
        pass

def main(args=None):
    print("Node: task_follow_me")
    rclpy.init(args=args)
    demo = Demo()
    executor = MultiThreadedExecutor()
    rclpy.spin(demo, executor)
    
    demo.destroy_node()
    rclpy.shutdown()
```

> 注意：
>
> 需要进行动作取消，一定要在服务端添加MultiThreadedExecutor()


