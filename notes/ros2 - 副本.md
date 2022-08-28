# ROS2开发者笔记——创建工程

## 版本

ROS2更新很快，至今为止已经发布了很多版本，根据发行时间各版本为：Ardent(2017.12)、Bouncy(2018.7)、Crystal(2018.12)、Dashing(2019.5)、Eloquent(2019.11)、Foxy(2020.6)、Galactic(2021.5)、Humble(2022.5)。

官方为每个版本维护了独立的参考文档：

- Dashing: [Tutorials — ROS 2 Documentation: Dashing documentation](https://docs.ros.org/en/dashing/Tutorials.html)
- Foxy: [Tutorials — ROS 2 Documentation: Foxy documentation](https://docs.ros.org/en/foxy/Tutorials.html)

在ROS2的学习中，同样的代码可能无法在不同版本上运行，所以一定注意你机器上安装的ROS2版本，避免造成`明明代码在其他机器上可以运行为什么在我机器上无法运行的问题`。所以写程序最要参考对应版本的文档说明。

## 参考文档

[ROS 2 · GitHub](https://github.com/ros2)

### API-C++

https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Duration.html

### API-python

https://docs.ros2.org/dashing/api/rclpy/index.html

## 安装

这里我们安装ros2-foxy!

### 设置编码

```shell
$sudo locale-gen en_US en_US.UTF-8
$sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$export LANG=en_US.UTF-8
```

### 设置软件源

```shell
$sudo apt update 
$sudo apt install curl gnupg2 lsb-release
###
# 下面这个官方给的命令会报错“gpg: no valid OpenPGP data found”
$curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# 请使用下面这个代替上面命令
$ curl http://repo.ros2.org/repos.key | sudo apt-key add - 
### 添加源
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

### 更新源

```shell
$sudo apt update
###若出现问题“N: Skipping acquire of configured file 'main/binary-i386/Packages' as repository 'http://packages.ros.org/ros2/ubuntu focal InRelease' doesn't support architecture 'i386'”
###该命令在树莓派arm64上没有出现，可能是不同CPU硬件架构区别导致的。
###解决方法：
"""
$ cd /etc/apt/sources.list.d
$ sudo gedit ros2-latest.list
# 打开文本后出现:
deb http://packages.ros.org/ros2/ubuntu bionic main
# 在deb后插入[arch=amd64]变成:
deb [arch=amd64] http://packages.ros.org/ros2/ubuntu bionic main
# 保存并关闭
# 再次更新便可
sudo apt update
"""
```

### 安装不同版本

```shell
### 
$sudo apt install ros-foxy-desktop
###
$sudo apt install ros-foxy-ros-base
```

### 安装自动补全工具

```shell
$ sudo apt install python3-argcomplete
```



### 设置环境变量

```shell
alex@alex-xiaomi:/mnt/c/Users/alex$ ls /opt/ros/
foxy/   noetic/
alex@alex-xiaomi:/mnt/c/Users/alex$ source /opt/ros/foxy/setup.bash
```

如果有安装ros1则会有提示一个警告

> ROS_DISTRO was set to 'noetic' before. Please make sure that the environment does not mix paths from different distributions.

这个时候需要把ROS的环境变量去掉，增加ROS2的环境变量。

设置下`~/.bashrc`文件如下：

```bash
#source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
```

### 测试

运行一个talker

```shell
alex@alex-xiaomi:/mnt/c/Users/alex$ ros2 run demo_nodes_cpp talker
[INFO] [1628004383.847069300] [talker]: Publishing: 'Hello World: 1'
[INFO] [1628004384.846837500] [talker]: Publishing: 'Hello World: 2'
```

如果要运行python版本的demo，将**demo_nodes_cpp**设置为**demo_nodes_py**即可。

运行一个listener

```shell
alex@alex-xiaomi:/mnt/c/Users/alex$ source /opt/ros/foxy/setup.bash
alex@alex-xiaomi:/mnt/c/Users/alex$ ros2 run demo_nodes_cpp listener
[INFO] [1628004397.878527800] [listener]: I heard: [Hello World: 15]
```

### 卸载

```shell
$sudo apt remove ros-foxy-* 
$sudo apt autoremove
```

### 问题和解决

### colcon build问题

1. setuptools相关

**问题：**

报错如下：

```shell
/home/alex/.local/lib/python3.6/site-packages/setuptools/dist.py:720: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  % (opt, underscore_opt)
  
Finished <<< pyServerClient [0.67s]

Summary: 2 packages finished [0.77s]
  2 packages had stderr output: pyServerClient py_pubsub
```

**解决：**

这个问题应该是setuptools安装的时候版本不匹配，可以卸载并安装最新的。

```shell
###卸载
$ python3 -m pip uninstall pip setuptools wheel
###重新安装
$ sudo apt-get --reinstall install python3-setuptools python3-wheel python3-pip
```

### 在conda中使用ros2

如果有的python安装包在conda环境下，例如在配置了一个跟踪检测的conda环境，希望在该环境下启动ros2能够调用次环境下的功能包，那么请在`~/.bahsrc`添加如下：

```shell
export PYTHONPATH=/home/alex/miniconda3/envs/alex/lib/python3.6/site-packages:$PYTHONPATH
```



## ROS2目录结构

工作空间主要包含的文件夹有：`src/`, `install/`,  `build/`, `log/`。

文件结构如下：

```shell
|--workspace_folder/
    |--src/
      |--package_cpp/
          |--CMakeLists.txt
          |--package.xml
      |--package_cpp/
      	  |--package_py/
      	      |--node1.py
      	      |--node2.py
          |--resource/package_cpp
          |--test
              |--test_copyright.py
              |--test_flake8.py
              |--test_pep257.py
          |--setup.py
          |--setup.cfg
          |--package.xml  #作者信息、功能包描述、指明需要依赖的其他包
	|-- install/
	  |-- package1_name/
	      |-- lib/
	          |-- package1_name/
	              |-- node1               # 编译生成的对应节点的可执行二进制文件
	          |-- python3.6/package1_name/
	              |-- node1.py            # 编译后拷贝过来的节点程序文件，会有节点可执行二进制文件调用
	          
```



## 基本操作

### 创建工作空间

#### 建立工程目录

```shell
alex@alex-xiaomi:/mnt/d/WSL/ros2_learn$ mkdir -p gyj_ws/src
alex@alex-xiaomi:/mnt/d/WSL/ros2_learn$ cd gyj_ws/src/
```

#### 安装编译工具calcon

```shell
sudo apt install python3-colcon-common-extensions
#或者
sudo apt update && sudo apt install -y \
build-essential \
cmake \
git \
python3-colcon-common-extensions \
python3-pip \
python-rosdep \
python3-vcstool \
wget
```

#### 编译

```shell
alex@alex-xiaomi:/mnt/d/WSL/ros2_learn/gyj_ws$ colcon build

Summary: 0 packages finished [0.15s]
```

colcon build后边还可以跟一些常用的参数：  

1. --packages-up-to ：编译指定的功能包，而不是整个工作空间
2. --symlink-install ：节省每次重建python脚本的时间
3. --packages-select： 选择要编译的package
4. --event-handlers console_direct+ ：在终端中显示编译过程中的详细日志

### 创建发布者和订阅者

#### 创建功能包

```shell
alex@alex-xiaomi:/mnt/d/WSL/ros2_learn/gyj_ws/src$ ros2 pkg create --build-type ament_python py_pubsub
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

```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

 设置程序入口

 接下来打开setup.py文件，同样需要补充以下内容： 

```
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',

```

  然后在 entry_points 下添加如下内容： 

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],},

```

 检查setup.cfg文件

 setup.cfg文件中内容是自动添加的，可以打开看下，内容如下： 

```
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
rosdep install -i --from-path src --rosdistro foxy -y
```

#### 运行测试

1、打开一个终端，设置工作空间环境变量，运行发布节点

```shell
alex@alex-xiaomi:/mnt/d/WSL/ros2_learn/gyj_ws$ . install/setup.bash
```

2、再打开一个终端，运行订阅节点

```
. install/setup.bash   
ros2 run py_pubsub listener
```

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



#### 通过ymal文件传参

```shell
$ ros2 run demo_nodes_cpp talker __params:=demo_params.yaml
```



### 命令行中获取运行节点的参数

```shell
$ ros2 param get /AutoAttacking "KP_drone_yaw"
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




