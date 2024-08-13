# 时间操作-cpp

## 时钟类型

ROS2中定义了三种时钟：RCL_SYSTEM_TIME、RCL_STEADY_TIME、RCL_ROS_TIME。

- RCL_SYSTEM_TIME

  默认是使用RCL_SYSTEM_TIME。它和C++中的std::chrono::system_clock是一样的，即系统时间。ROS2提供的消息打印`RCLCPP_INFO()`函数中的用的就是系统时间。

- RCL_STEADY_TIME

  一般用于对程序段执行时间进行计时。

- RCL_ROS_TIME

ROS2定义了枚举类型**rcl_clock_type_t**：

```c++
typedef enum rcl_clock_type_t
{
  /// Clock uninitialized
  RCL_CLOCK_UNINITIALIZED = 0,
  /// Use ROS time，1
  RCL_ROS_TIME,
  /// Use system time，2
  RCL_SYSTEM_TIME,
  /// Use a steady clock time，3
  RCL_STEADY_TIME
} rcl_clock_type_t;
```



## 相关类

### **rclcpp::Clock**

参考：[dashing](https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Clock.html)

时钟类

#### 构造函数

只有一个构造函数，Clock (rcl_clock_type_t clock_type=RCL_SYSTEM_TIME)

```c++
```



#### 成员函数

其中有个函数`now()`，返回值为Time类型。

### **rclcpp::Time**

参考：[dashing](https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Time.html)

时刻类

#### 构造函数

     *  通过时间组成的两部分构造
          *
     Time (int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t clock_type=RCL_SYSTEM_TIME)
    时间由两个部分组成，最终的时间为seconds+nanoseconds
    
    ```c++
    rclcpp::Time time1(100, 123, RCL_SYSTEM_TIME);
    ```


​    

     * 通过ROS2内置消息类型进行构造
          *
      Time (const builtin_interfaces::msg::Time &time_msg, rcl_clock_type_t clock_type=RCL_ROS_TIME)
    
    ```c++
    builtin_interfaces::msg::Time stamp;
    stamp.sec     = 100;
    stamp.nanosec = 123;
    rclcpp::Time time2(stamp);
    ```


​    

     * 通过结构体rcl_time_point_t进行构造
          *
    Time (const builtin_interfaces::msg::Time &time_msg, rcl_clock_type_t clock_type=RCL_ROS_TIME)
    
     ```c++
     rcl_time_point_t tp;
     tp.nanoseconds = 100123456789;
     tp.clock_type = RCL_SYSTEM_TIME;
     rclcpp::Time time3(tp);
     ```

#### 成员函数

- [nanoseconds](https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Time.html#a0bab304d7b9e92dc568b069684275fe5) () const

  返回[rcl_time_point_value_t](http://docs.ros2.org/dashing/api/rcl/time_8h.html#a504861cd584105c7ddfea1dd6539f8e8)类型值，本身是int64_t，代表从Unix epoch起始的时间，单位是纳秒，用于精确计时。

- [seconds](https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Time.html#a586e6c368730e6847519b7cd928359f5) () const

  返回double类型值，表示从Unix epoch起始的时间，单位是秒。

两个成员函数是等价的，只不过返回的时间单位不一样而已。

> *Unix* 时间戳是从1970年1月1日（UTC/GMT的午夜）开始所经过的秒数

### **rclcpp::Duration**

参考：[dashing](https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Duration.html)

时间间隔类。

#### 成员函数

- [nanoseconds](https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Time.html#a0bab304d7b9e92dc568b069684275fe5) () const

  返回[rcl_time_point_value_t](http://docs.ros2.org/dashing/api/rcl/time_8h.html#a504861cd584105c7ddfea1dd6539f8e8)类型值，本身是int64_t，代表从Unix epoch起始的时间，单位是纳秒，用于精确计时。

- [seconds](https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Time.html#a586e6c368730e6847519b7cd928359f5) () const

  返回double类型值，表示从Unix epoch起始的时间，单位是秒。

时间间隔和时刻可进行加减运算。

## 仿真时间

`ROS2`中存在两种时间戳。一种是实际的物理系统时间，另一种是仿真时间。仿真时间通常是`Gazebo`发出的`/clock`话题。

当需要在仿真环境中测试程序时，要将`use_sim_time`置为`True`，置位`use_sim_time`可以写在`launch`文件中。如下所示：

```python
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='time_api_test',
            executable='time_api_test',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[],
            output='screen'),
    ])
```

仿真时间可以通过如下函数获取：

- this->get_clock()->now()

- this->now()

> 如果use_sim_time=false，则以上两个函数获取的时间与rclcpp::Clock().now()一样，都是系统时间。



## Header

topic消息中，时间一般都会用std_msgs/Header表示。

其格式如下：

```shell
$ ros2 interface show std_msgs/msg/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
```

其中stamp为：

```shell
$ ros2 interface show builtin_interfaces/msg/Time
# Time indicates a specific point in time, relative to a clock's 0 point.

# The seconds component, valid over all int32 values.
int32 sec
# The nanoseconds component, valid in the range [0, 10e9).
uint32 nanosec
```



## 示例

### 消息时间戳赋值

- foxy

```c++
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "std_msgs/msg/imu.hpp"

rclcpp::TimeSource ts(node);
rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
ts.attachClock(clock);

class TestNode()
{
public:
	void test()
    {
        auto msg = sensor_msgs::msg::Imu();
        //
        msg.header.stamp = clock->now();  
        // 通过本节点get_clock()得到时间
        msg.header.stamp = this->get_clock()->now();
        // 通过本节点now()直接得到时间
        msg.header.stamp = this->now();
        // 通过Clock实例化对象得到时间
        msg.header.stamp = rclcpp::Clock().now();
    }
}

```



如果是在继承了的类中使用，则可以通过成员函数`now()`直接得到：

```c++
class SimplePublisherNode : public rclcpp::Node
{
    message->header.stamp = this->now();
}
```



### 计算程序段运行时间

```shell
rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

auto start_time = steady_clock_.now();
//do something ...
auto cycle_duration = steady_clock_.now() - start_time;

RCLCPP_INFO(get_logger(), "Cost %.4f s", cycle_duration.seconds());
```



### 与chrono

**rclcpp::Time与std::chrono::time_point**

```c++
//Time->time_point
RCLCPP_INFO(node->get_logger(), "convert rclcpp::Time to std::chrono::time_point");
rclcpp::Time time = node->now();
std::chrono::time_point<std::chrono::nanoseconds> tp(std::chrono::nanoseconds(time.nanoseconds()));
RCLCPP_INFO(node->get_logger(), "time point: count=%ld", tp.time_since_epoch().count());

//time_point->Time
RCLCPP_INFO(node->get_logger(), "convert std::chrono::time_point to rclcpp::Time");
std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp2 = std::chrono::system_clock::now();
rclcpp::Time time2(rcl_time_point_t{tp2.time_since_epoch().count(), RCL_SYSTEM_TIME});
RCLCPP_INFO(node->get_logger(), "time: sec=%f, nanosec=%ld", time2.seconds(), time2.nanoseconds());
```



## 时间同步

https://blog.csdn.net/qq_41943585/article/details/89679198?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-89679198-blog-114752452.pc_relevant_multi_platform_whitelistv1_exp2&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-89679198-blog-114752452.pc_relevant_multi_platform_whitelistv1_exp2&utm_relevant_index=1



```bash
ros2 rmw_uros_sync_session
```

