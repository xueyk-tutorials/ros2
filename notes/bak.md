### action

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

## 消息

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

## 服务

判断服务端是否存在

```python
### 创建客户端
self.cli_tracker_cmd = self.create_client(TrackerCmd, 'set_tracker')

### 判断服务端是否存在
if self.cli_tracker_cmd.service_is_ready():
    request = TrackerCmd.Request()
```

## ros2 run



```shell
$ ros2 run demo_nodes_cpp talker __ns:=/demo __node:=my_talker chatter:=my_topic

$ ros2 run demo_nodes_cpp talker __params:=demo_params.yaml
```



### 参数传递

在运行节点时，可以通过命令行给节点传参。

```shell
$ ros2 run flocking multirotor_keyboard_control 1 2 3 4
```

在节点中可以通过`sys.argv`获取传入的参数

```python
class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        print(sys.argv)
        arg1 = sys.argv[1]
        arg2 = sys.argv[2]
"""
### print
['/home/alex/Desktop/demo_flocking/install/flocking/lib/flocking/multirotor_keyboard_control', '1', '2', '3', '4']
"""
```



## parameter

### 节点中参数使用

#### 声明

- 声明一个参数

```python
self.declare_parameter("address", "chengdu")
```

- 多个参数一起声明

```python
self.declare_parameters('', [('uav_type', 'iris'), ('leader_id', 0), ('uav_num', 6)])
```
#### 获取参数值

```shell
self.declare_parameter("address", "chengdu")
self.get_parameter("address").get_parameter_value().string_value
```

>其中参数的类型包括有：
>
>```shell
>bool_value
>integer_value
>double_value
>string_value
>byte_array_value
>bool_array_value
>integer_array_value
>double_array_value
>string_array_value
>```

**在使用时特别注意：**

1. 在声明变量时，一定要将默认值写规范，默认值类型一定要与参数类型一致，特别是获取参数类型是`double_value`时，声明变量也要写成浮点数。例如：

```python
self.declare_parameter("height", 30)       # wrong
self.declare_parameter("height", 30.0)     # right
self.get_parameter("height").get_parameter_value().double_value
```

2. 如果参数是string类型，launch文件中，不能给参数传递如'6'、'19'等纯数字字符串！

3. 如果参数是数组，一定要保证数组内的值类型一致！例如[1, 2, 3]或[1.2, 2.3, 4.5]等，不能不同类型的值放到一起，如[1, 2.0]会报错！

#### 判断参数是否存在

```python
if self.has_parameter("age"):
    pass
```



### 命令行给节点传参

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



#### 通过ymal文件传参

```shell
$ ros2 run demo_nodes_cpp talker __params:=demo_params.yaml
```



### 命令行中获取运行节点的参数

```shell
$ ros2 param get /AutoAttacking "KP_drone_yaw"
```



## launch

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

## namespace

namespace简单理解就是命名空间，在编程领域中，不同命名空间中的变量是可以同名的，不会有任何影响。在ROS中，消息的通信是必须通过话题名来建立，话题名可以被当做一个地址，为了避免节点类实例化并运行多个出现多个消息的话题名因同名冲突，可以在话题名再加上空间限制。

例如使用小海龟仿真时会有一个话题名为`/turtle1/pose`，这个`turtle1`就是namespace，这样当同一个窗口再出现一个小海龟，为了进行区分，把第二个小海龟的命名空间定义成`turtle2`就能将其同样的话题变成`/turtle2/pose`与第一个小海龟的话题区分开了。

为了研究下namespace如何影响话题名，我们做一些测试。

1. 测试一：使用`ros2 run node_name`的方式启动节点：

````python
### 话题定义（前缀不加/） ###                                                        
self.pub_talker = self.create_publisher(String, 'test/talker', 1) 
```
$ ros2 topic list
/parameter_events
/rosout
/test/talker
```
### 话题定义（前缀加/） ###   
```
$ ros2 topic list
/parameter_events
/rosout
/test/talker
```
````

> 节点中定义话题名称时加与不加前缀`/`都没有区别，

2. 测试使用launch启动节点，并且在launch中定义了**namespace**

例如launch文件如下：

```python
Node(
    package='learn_launch',
    node_namespace='my_ns',
    node_executable='namespace_test',
    node_name='namespace_test',
)
```

进行测试：

````python
### 话题定义（前缀不加/） ###                                                        
self.pub_talker = self.create_publisher(String, 'test/talker', 1) 
```
$ ros2 topic list
/my_ns/parameter_events
/my_ns/rosout
/my_ns/test/talker        # namespace对话题名进行了扩展和限定
/parameter_events
/rosout
```
### 话题定义（前缀加/） ###                                                        
self.pub_talker = self.create_publisher(String, '/test/talker', 1) 
```
$ ros2 topic list
/my_ns/parameter_events
/my_ns/rosout
/parameter_events
/rosout
/test/talker               # 话题名没有变化
```
````

> 也就是说如果节点内的话题名添加了`/`，那么**namespace**不起作用；
>
> 只有节点内的话题名没有添加前缀`/`，**namespace**才会生效，可以对话题名进行扩展和限定：

## rqt

在开发中，我们往往只写自己负责的部分功能代码，由于ROS2的通信需要成对的节点，举例来说你写完负责的接收或者发布节点，为了进行代码测试难道还得写另一个节点吗？或者等其他人把相应的节点开发完？NO, NO, NO，ROS2提供了这些功能，能够生成发布节点进行消息发布，或者接收节点进行消息接收，而且是可以基本可视化的！而这个功能就是**rqt**！

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



## 多机通信

对于需要多个计算机基于ROS2进行互相通信时，编辑`~/.bashrc`，在最后添加如下：

```shell
export ROS_DOMAIN_ID=1
```





## 创建发布

```python
self.node = rclpy.create_node('my_node')
self.pub = self.node.create_publisher(msg_type, topic)
```



## 时间

### rclpy.time.Time

[类源码](https://github.com/ros2/rclpy/blob/196669e539dd51bc56f9f4fdf6f0222d99480d0d/rclpy/rclpy/time.py#L138-L140)参考github。

```python
class Time():
    def to_msg(self):
        seconds, nanoseconds = self.seconds_nanoseconds()
        return builtin_interfaces.msg.Time(sec=seconds, nanosec=nanoseconds)
```

### builtin_interfaces.msg.Time

时间消息包含两个成员：`sec`，`nanosec`，即秒和纳秒，格式如下：

```shell
ros2 msg show builtin_interfaces/msg/Time 
int32 sec
uint32 nanosec
```



### 时间转换

- from `rclpy.time.Time` to `builtin_interfaces.msg.Time`

```python
header = Header()
header.stamp = node.get_clock().now().to_msg()
```

- from `builtin_interfaces.msg.Time`  to  `rclpy.time.Time` 

```python
header = Header()
header.stamp = self.get_clock().now().to_msg()
### 第一种, 使用from_msg方法
cur_time = Time.from_msg(header.stamp)
### 第二种，使用类实例化方法
cur_time = Time(seconds=header.stamp.sec, nanoseconds=header.stamp.nanosec)
```

- 与python的time模块

```python
last_time = self.get_clock().now()
sec, nanosec = last_time.seconds_nanoseconds()
last_time = sec + nanosec * 1e-9
last_time_py = time.time()
print(last_time, last_time_py)
time.sleep(0.001)
cur_time = self.get_clock().now()
sec, nanosec = cur_time.seconds_nanoseconds()
cur_time = sec + nanosec * 1e-9
print(cur_time, cur_time-last_time_py)
```



### 周期

```python
self.timer = self.create_timer(1, self.timer_callback)
def timer_callback(self):
    pass
```



### c版本

```c++
message->header.stamp = this->now();


// Compute latency
rclcpp::Time stamp(msg->header.stamp);
auto lat = std::chrono::nanoseconds((now - stamp).nanoseconds());
int lat_us = lat.count() / 1000;
```





## 日志

launch启动后会提示记录的文件路径，launch日志记录的位置如：

```shell
/home/alex/.ros/log/2021-11-03-16-18-06-525630-alex-Mi-Gaming-Laptop-15-6-20322
```

注意这个是一个文件夹，里面会包含一个`/launch.log`文件。

## setup.py

setup.py是python第三方库setuptools提供的一种打包和安装的规则，这个文件类似于CMakeLists.txt，能够将开发代码进行统一的打包和安装。

一个初始状态的setup.py如下：

```python
from setuptools import setup
package_name = 'learn_pkg'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='cetcs_jizhi_group@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

```

> 这个是从一个名称为**learn_pkg**的ros2包中的setup.py文件拷贝而来！

- **packages**

用于进行package的打包。

例如有个package如下：

```python
|-- src/learn_pkg/learn_pkg/
    |-- Drivers/
        |-- __init__.py
        |-- pod.py
        |-- comm.py
        |-- drone.py
```

那么打包设置为：

```python
packages = [package_name, 'Drivers'],
```

**如何在其他文件中引用Drivers呢？**

通过package的设置后，ROS2进行工程编译，会把Drivers打包到新生成的install目录下，并且将这个目录下的`install/learn_pkg/lib/python3.6/site-packages`添加到python环境变量中，就相当于这个Drivers安装在了本地计算机！其他python文件进行引用时使用：

```shell
import learn_pkg.Drivers.pod as pod
```

- **data_files**

用于进行依赖文件的打包，例如配置文件、图片等等。

data_files是个元组列表，其中的元组内有两个元素。

例如ros包目录如下：

```shell
|-- src/learn_pkg/learn_pkg/
    |-- Drivers/
    |-- config.txt
    |-- my_node.py
```

**如果希望把`config.txt`作为配置文件打包到安装目录，可设置如下:**

```python
import os
data_files=[
        (os.path.join('lib/python3.6/site-packages', package_name), package_name + 'config.txt'),
    ],
```

这时对工程进行编译后，生成的install目录如下：

```python
|-- ROS2_WS/
	|-- install/
		|-- learn_pkg/
			|-- libs/
				|-- learn_pkg/
				|-- python3.6/site-packages/
					|-- learn_pkg/
                    	|-- my_node.py
                        |-- config.txt
```

**那如何在启动my_node.py节点时加载config.txt呢？**

由于ros2启动节点时，就是运行的安装目录下的python文件`ROS2_WS/install/learn_pkg/lib/python3.6/site-packages/learn_pkg/my_node.py`，而且配置文件与这个python文件的相对路径是明确的，那么我们可以通过获取python文件的绝对路径后，再通过相对路径找到配置文件。例如my_node.py可以添加如下路径相关的操作代码：

```python
# 通过__file__获取运行的python文件。
path = os.path.join(os.path.dirname(__file__), ‘config.txt’)
```



## 示例

### 在节点中引用文件

如果某个功能涉及到的功能较多，需要多个文件才能完成功能实现，则这些文件夹需要放到一个文件夹下，然而在节点中又需要导入这些功能文件，这时直接使用import是无效的，因为ros2节点运行时的可执行文件是在`install`文件夹下的，无法找到这些功能文件。

例如目录结构（未编译之前）如下：

```shell
|-- ROS2_WS/
	|-- src/
		|-- learn_pkg/
			|-- learn_pkg/
				|-- my_libs/
					|-- functions.py
				|-- my_node.py
			|-- setup.py
```

节点程序`my_node.py`如下：

```python
import rclpy
from rclpy.node import Node
### 导入库中的所有功能包
from my_libs import *

def main(args=None):
    print("INFO: lib_caller.py")
    rclpy.init(args=args)
    node = Node('lib_caller')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

这时候就需要`setup.py`进行相应配置。

```python
from setuptools import setup
import os
from glob import glob

package_name = 'learn_pkg'

setup(
data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', *.launch.py'))),
        ######################
        ### 将my_libs库中的所有文件拷贝至安装目录下
        ######################
        (os.path.join('lib', package_name + '/my_libs'), glob(package_name + '/my_libs/*.py')),   
    ],
)
```

编译后的目录结构如下：

```shell
|-- ROS2_WS/
	|-- install/
		|-- learn_pkg/
			|-- lib/
				|-- my_libs/      # 拷贝过来的库
				|-- my_node       # 编译后的执行文件
			|-- share/
	|-- src/
		|-- learn_pkg/
			|-- learn_pkg/
			|-- setup.py
```





### 通过命令行传参

如下的例子将talker节点变为my_talker节点。发布的topic从chatter改名为my_topic。用于限定topic/service的名字空间被设为/demo，这样使用topic时就变为/demo/my_topic，而非全局化的/my_topic。

```shell
ros2 run demo_nodes_cpp talker __ns:=/demo __node:=my_talker chatter:=my_topic
```

