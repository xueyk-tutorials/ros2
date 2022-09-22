# ROS2动作

## C++

## python

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

