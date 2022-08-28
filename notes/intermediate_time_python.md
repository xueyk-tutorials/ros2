# 时间操作-python

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

