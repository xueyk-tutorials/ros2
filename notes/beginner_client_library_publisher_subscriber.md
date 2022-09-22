# ROS2发布与订阅

## C++

## python

不要在`wall_timer`的回调函数中直接读取摄像头！会造成该节点的消息订阅无法正常工作。

### 创建功能包

```shell
$ ros2 pkg create --build-type ament_python py_pubsub
```

### 创建发布者

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

### 创建订阅者

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

### 编译

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

### 运行测试

1、打开一个终端，设置工作空间环境变量，运行发布节点

```shell
$ . install/setup.bash
```

2、再打开一个终端，运行订阅节点

```shell
$ . install/setup.bash   
$ ros2 run py_pubsub listener
```

## 

## 服务

判断服务端是否存在

```python
### 创建客户端
self.cli_tracker_cmd = self.create_client(TrackerCmd, 'set_tracker')

### 判断服务端是否存在
if self.cli_tracker_cmd.service_is_ready():
    request = TrackerCmd.Request()
```





## C++服务编程

一定要注意，含有service的节点一定要作为参数传入rclcpp::spin(your_node)中才可以，否则服务无法启动！





get_node_services_interface()





## 使用匿名函数作为回调

### lambda函数

匿名函数有函数体，没有函数名。基本形式如下：

`[capture](parameters)->return-type {body}`

[]叫做捕获说明符，表示一个lambda表达式的开始。接下来是参数列表，即这个匿名的lambda函数的参数，->return-type表示返回类型，如果没有返回类型，则可以省略这部分。最后就是函数体部分了。

简单示例：

```shell
auto func = [] () { cout << "hello,world"; };
func(); // now call the function
```





```c++
timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>("/fmu/timesync/out",
    10,
    [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
        timestamp_.store(msg->timestamp);
    });
```

