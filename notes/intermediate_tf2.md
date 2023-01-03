# tf2

## 常用命令

- 显示tf树并保存至PDF文件。

```bash
$ ros2 run tf2_tools view_frames.py
```

- 查看两个坐标系变换关系

```shell
$ ros2 run tf2_ros tf2_echo earth body
```



## API

### quaternion

在tf2中，四元数表示为 `(x, y, z, w)`，也就是最后一个为w。

实例化对象：

```
tf2::Quaternion q;
```

#### q.x(), q.y(), q.z(), q.w()

获取四元数中的值

#### q.setRPY(roll, pitch, yaw)

通过欧拉角设置四元数，这里roll、pitch、yaw为分别绕X，Y，Z轴旋转得到的角度。

这里欧拉角与PX4飞控定义一致，通过欧拉角得到的四元素为$q_b^e$。

#### q.normalize()

归一化

#### 四元素与欧拉角

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



#### 四元素与geometry_msgs

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



## 功能包

### tf2_ros

#### static_transform_publisher节点

通过欧拉角得到转换关系

```shell
$ ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
```

通过四元素得到转换关系

```shell
$ ros2 run tf2_ros static_transform_publisher x y z qx qy qz qw frame_id child_frame_id
```

> 这里frame_id表示参考坐标系（例如大地坐标系earth），child_frame_id表示子坐标系（例如无人机机体坐标系body），故：
>
> - 这里的(x,y,z)是child_frame_id坐标系原点在frame_id坐标系下的坐标！
> - 这里的roll, pitch, yaw是child_frame_id坐标系在frame_id坐标系下进行的旋转！

示例

```shell
$ ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0.5 earth body
[INFO] [1671197004.238799528] [static_transform_publisher_UwLBOX7fSPIDXLbq]: Spinning until killed publishing transform from 'earth' to 'body'

<pre>$ ros2 topic echo /tf_static
transforms:
- header:
    stamp:
      sec: 1671197077
      nanosec: 6663728
    frame_id: earth
  child_frame_id: body
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.24740395925452294
      y: 0.0
      z: 0.0
      w: 0.9689124217106447
---
```



#### tf2_echo

可以通过`ros2 run tf2_ros tf2_echo  <frame1> <frame2>`查看两个坐标系的相对关系。这里是frame1作为参考坐标系，求frame2在frame1下的偏移和旋转。

```shell
$ ros2 run tf2_ros tf2_echo earth body
```

> 求body在earth frame下的旋转和平移。
>
> 输出的四元素为$q_b^e$



### tf2_tools

显示tf树并保存至PDF文件。

```bash
ros2 run tf2_tools view_frames.py
```
