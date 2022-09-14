# ROS2开发者笔记——Docker

## 安装

### 下载镜像

参考：http://docs.ros.org/en/foxy/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html

```shell
## 下载镜像
$ docker pull osrf/ros:foxy-desktop
```

### 运行

#### 示例：基础

```shell
## 运行
$ docker run -it osrf/ros:foxy-desktop
```

> 以上命令启动时会自动运行/ros_entrypoint.sh脚本。

#### 示例：数据卷挂载

```shell
## 数据卷挂载
$ docker run -it -v /home/alex/docker_volume:/home osrf/ros:foxy-desktop
```

#### 示例：在两个独立容器内运行

```shell
### 打开第一个终端
$docker run -it --rm osrf/ros:foxy-desktop ros2 run demo_nodes_cpp talker
### 打开第二个终端
$ docker run -it --rm osrf/ros:foxy-desktop ros2 run demo_nodes_cpp listener
```

#### 示例：两个终端进入同一个容器

```shell
### 打开第一个终端，启动容器
$ docker run -it osrf/ros:foxy-desktop
### 打开第二个终端，进入已经运行的同一个容器
$ docker exec -it f34ce6aab6dc /bin/bash
root@f34ce6aab6dc:/# . ros_entrypoint.sh   
```



## 多节点通信

-p 3306:3306 -p 8080:8080



### 主机容器下ros2节点与其他主机下节点通信

**环境说明**

节点1：运行在主机1的容器下，例如在笔记本启动docker容器，并在容器中运行节点；

节点2：运行在主机2下，例如在树莓派上直接启动节点；

**通信方案**

运行docker容器时，使用host网络模式！

**操作流程**

主机1操作：

1. 启动容器

```shell
$ docker run -it --net=host --pid=host --name multi_nodes_test osrf/ros:foxy-desktop
```

在容器中添加变量`ROS_DOMAIN_ID=1`，确保不同主机之间可以发现。

```shell
root@alex-Mi-Gaming-Laptop-15-6:/# export ROS_DOMAIN_ID=1
```



主机2操作：

1. 查看当前topic

   ```shell
   ubuntu@ubuntu:~$ ros2 topic list
   /chatter
   /parameter_events
   /rosout
   ```

2. 打印topic

   ```shell
   ubuntu@ubuntu:~$ ros2 topic echo /chatter
   data: 'Hello World: 11'
   ---
   data: 'Hello World: 12'
   ---
   data: 'Hello World: 13'
   ---
   data: 'Hello World: 14'
   ```

   

参考：[Connecting Remote Robots Using ROS2, Docker & VPN | Husarnet](https://husarnet.com/blog/ros2-docker)

## 启动容器后运行ROS2

下载的ROS2 docker镜像，一般都会提供ROS入口脚本（容器根目录下的`ros_entrypoint.sh`），该脚本是容器启动的默认脚本，用于添加ROS2环境变量，一般内容如下：

```shell
#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
exec "$@"
```

### 运行单个节点

创建好ROS2容器，可以直接输入运行talker节点的命令：

```shell
$ docker run -it osrf/ros:foxy-desktop ros2 run demo_nodes_cpp talker
```



### 运行launch（方法一）

通过在ROS2容器启动脚本`ros_entrypoint.sh`中添加用户的ROS2工程的环境变量，通过命令行启动。

1. 首先启动一个ROS2容器

   ```shell
   docker run -it --rm osrf/ros:foxy-desktop
   ```

2. 在容器中部署ROS工程

   我们需要实现创建好ROS2工程，添加对应的launch文件，例如我们创建了工程，并且新增了helloworld包，以及在该包的launch目录下添加了`helloworld_talker.launch.py`。

3. 将工程的环境变量添加至启动脚本`ros_entrypoint.sh`中

   ```shell
   #!/bin/bash
   set -e
   
   # setup ros2 environment
   source "/opt/ros/$ROS_DISTRO/setup.bash" --
   # run your nodes!
   . /home/test/install/setup.bash                      # 添加工程环境变量
   exec "$@"
   
   ```

4. 生成新的镜像

   我们在启动的容器中进行ROS2工程部署之后，将该容器**commit**生成一个新的镜像，例如镜像名称为ros2-helloworld:v1

5. 运行新的镜像容器

   ```shell
   $ docker run -it ros2-helloworld:v1 ros2 launch helloworld helloworld_talker.launch.py
   ```



完成上面操作后，其他一些可能使用的操作有：

- 在新终端中进入正在运行的容器

```shell
$ docker exec -it <容器ID> /bin/bash.sh
```

- 启动停止的容器并运行ros

```shell
$ docker exec -it <容器ID> /ros_entrypoint.sh
```

**容器启动后，ctrl+c会直接停止节点和容器！**

### 运行launch（方法二）

与方法一基本差不多，只不过是将launch启动命令一起添加至启动脚本`ros_entrypoint.sh`中

```shell
#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
# run your nodes!
. /home/test/install/setup.bash                      # 添加工程环境变量
ros2 launch helloworld helloworld_talker.launch.py   # 添加launch
exec "$@"
```

这种方法，启动容器后，ctrl+c后，节点停止，容器继续运行！

### 运行launch（方法三）

这里我们通过在容器根目录中自己创建启动脚本的方式，运行launch。

1. 首先启动一个ROS2容器

   ```shell
   docker run -it --rm osrf/ros:foxy-desktop
   ```

2. 在容器中部署ROS工程

   我们需要实现创建好ROS2工程，添加对应的launch文件，例如我们创建了工程，并且新增了helloworld包，以及在该包的launch目录下添加了`helloworld_talker.launch.py`。

   在根目录创建一个启动脚本`start_ros2.sh`，添加如下：

   ```shell
   # 添加package路径
   . /home/test/install/setup.bash
   # 运行脚本
   ros2 launch helloworld helloworld_talker.launch.py
   ```

   给该启动脚本赋运行权限

   ```shell
   chmod u+x start_ros2.sh
   ```

3. 生成新的镜像

   我们在启动的容器中进行ROS2工程部署之后，将该容器**commit**生成一个新的镜像，例如镜像名称为ros2-helloworld:v3

4. 运行新的镜像容器

   ```shell
   docker run -it --rm ros2-helloworld:v3 /start_ros2.sh
   ```

5. 在新终端中进入容器

   ```shell
   $ docker exec -it <容器ID> /start_ros2.sh
   ```



## 容器可视化

https://mobaxterm.mobatek.net/

