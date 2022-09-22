# ROS2服务

## C++

一定要注意，含有service的节点一定要作为参数传入rclcpp::spin(your_node)中才可以，否则服务无法启动！





get_node_services_interface()

## python

判断服务端是否存在

```python
### 创建客户端
self.cli_tracker_cmd = self.create_client(TrackerCmd, 'set_tracker')

### 判断服务端是否存在
if self.cli_tracker_cmd.service_is_ready():
    request = TrackerCmd.Request()
```


