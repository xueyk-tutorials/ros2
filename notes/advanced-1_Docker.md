## ROS2开发者笔记——Docker

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



## 容器可视化

https://mobaxterm.mobatek.net/

