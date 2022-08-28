# tf2





## quaternion

在tf2中，四元数表示为 `(x, y, z, w)`，也就是最后一个为w。

实例化对象：

```
tf2::Quaternion q;
```

### API

#### q.x(), q.y(), q.z(), q.w()

获取四元数中的值

#### q.setRPY(roll, pitch, yaw)

通过欧拉角设置四元数，这里roll、pitch、yaw为分别绕X，Y，Z轴旋转得到的角度。与PX4飞控定义一致。

#### q.normalize()

归一化

### 与欧拉角

```c++
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//欧拉角转四元数
  tf2::Quaternion orientation;
  float yaw=1;
  orientation.setRPY(0.0, 0.0, yaw);

//四元数转欧拉角
    tf2::Quaternion imu_quat(
      imu_data.orientation.x,
      imu_data.orientation.y,
      imu_data.orientation.z,
      imu_data.orientation.w);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf2::Matrix3x3 m(imu_quat);
    m.getRPY(roll, pitch, yaw);//进行转换

```



### 与geometry_msgs

**tf2::Quaternion**对应的ROS2消息为**geometry_msgs::msg::Quaternion**，二者可以通过如下方式转换：

```c++
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
...

tf2::Quaternion tf2_quat, tf2_quat_from_msg;
tf2_quat.setRPY(roll, pitch, yaw);
// Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

// Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
tf2::convert(msg_quat, tf2_quat_from_msg);
// or
tf2::fromMsg(msg_quat, tf2_quat_from_msg);
```



