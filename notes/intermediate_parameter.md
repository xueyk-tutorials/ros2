# 参数

## 命令行参数传递

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



## 节点参数设置

### 声明

- 声明一个参数

```python
self.declare_parameter("address", "chengdu")
```

- 多个参数一起声明

```python
self.declare_parameters('', [('uav_type', 'iris'), ('leader_id', 0), ('uav_num', 6)])
```
### 获取参数值

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

### 判断参数是否存在

```python
if self.has_parameter("age"):
    pass
```



## 命令行给节点传参

参考：http://docs.ros.org/en/foxy/How-To-Guides/Node-arguments.html

### 直接设定参数

命令格式为：

```shell
$ ros2 run package_name executable_name --ros-args -p <param_name>:=<param_value>
```

例如：

```shell
ros2 run demo_nodes_cpp parameter_blackboard --ros-args -p some_int:=42 -p "a_string:=Hello world" -p "some_lists.some_integers:=[1, 2, 3, 4]" -p "some_lists.some_doubles:=[3.14, 2.718]"
```



### 通过ymal文件传参

```shell
$ ros2 run demo_nodes_cpp talker __params:=demo_params.yaml
```



### 命令行中获取运行节点的参数

```shell
$ ros2 param get /AutoAttacking "KP_drone_yaw"
```



### 通过命令行传参

如下的例子将talker节点变为my_talker节点。发布的topic从chatter改名为my_topic。用于限定topic/service的名字空间被设为/demo，这样使用topic时就变为/demo/my_topic，而非全局化的/my_topic。

```shell
ros2 run demo_nodes_cpp talker __ns:=/demo __node:=my_talker chatter:=my_topic
```

