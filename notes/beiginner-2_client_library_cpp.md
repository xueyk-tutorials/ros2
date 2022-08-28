# ROS2 client library(C++)



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

