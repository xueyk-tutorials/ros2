# namespace

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

## 