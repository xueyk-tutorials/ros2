

## qos_basic
编译
```shell
$ colcon build --packages-select qos_basic
```
启动节点：
```shell
$ ros2 run qos_basic qos_msg_pub
```

设置qos=rmw_qos_profile_sensor_data:
```shell
$ ros2 topic info /qos/string --verbose
Type: std_msgs/msg/String

Publisher count: 1

Node name: Demo
Node namespace: /
Topic type: std_msgs/msg/String
Endpoint type: PUBLISHER
GID: 01.0f.fe.53.8a.04.00.00.01.00.00.00.00.00.12.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 0
```

设置qos后：
```shell
$ ros2 topic info /qos/string --verbose
Type: std_msgs/msg/String

Publisher count: 1

Node name: Demo
Node namespace: /
Topic type: std_msgs/msg/String
Endpoint type: PUBLISHER
GID: 01.0f.fe.53.e1.03.00.00.01.00.00.00.00.00.12.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 20000000 nanoseconds
  Deadline: 30000000 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 0
```