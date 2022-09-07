# ROS2开发者笔记——安装

## 版本

ROS2更新很快，至今为止已经发布了很多版本，根据发行时间各版本为：Ardent(2017.12)、Bouncy(2018.7)、Crystal(2018.12)、Dashing(2019.5)、Eloquent(2019.11)、Foxy(2020.6)、Galactic(2021.5)、Humble(2022.5)。

官方为每个版本维护了独立的参考文档：

- Dashing: [Tutorials — ROS 2 Documentation: Dashing documentation](https://docs.ros.org/en/dashing/Tutorials.html)
- Foxy: [Tutorials — ROS 2 Documentation: Foxy documentation](https://docs.ros.org/en/foxy/Tutorials.html)

在ROS2的学习中，同样的代码可能无法在不同版本上运行，所以一定注意你机器上安装的ROS2版本，避免造成`明明代码在其他机器上可以运行为什么在我机器上无法运行的问题`。所以写程序最要参考对应版本的文档说明，在最下方可以选择版本。

## 在线安装ROS2

这里我们以Ubuntu20.04下安装ros2-foxy为例！

为了加快安装速度，建议将Ubuntu软件源设置为国内镜像。

### 设置编码

```shell
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8
```



### 下载GPGkey

```shell
$ sudo apt update 
$ sudo apt install curl gnupg2 lsb-release
### 
# 选型一：下面这个官方给的命令会报错“gpg: no valid OpenPGP data found”
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# 选项二：请使用下面这个代替上面命令（推荐）
$ curl http://repo.ros2.org/repos.key | sudo apt-key add - 
```

### 设置软件源

```shell
### 添加源
# 选项一：使用默认源
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
# 选项二：使用清华源（推荐）
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://mirror.tuna.tsinghua.edu.cn/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

### 更新源

```shell
$ sudo apt update
```

> **注意：**
>
> 若出现问题 “N: Skipping acquire of configured file 'main/binary-i386/Packages' as repository 'http://packages.ros.org/ros2/ubuntu focal InRelease' doesn't support architecture 'i386'”
> 该命令在树莓派arm64上没有出现，可能是不同CPU硬件架构区别导致的。
> 解决方法：
>
> ``` shell
> $ cd /etc/apt/sources.list.d
> $ sudo gedit ros2-latest.list
> 打开文本后出现:
> deb http://packages.ros.org/ros2/ubuntu bionic main
> 在deb后插入[arch=amd64]变成:
> deb [arch=amd64] http://packages.ros.org/ros2/ubuntu bionic main
> 保存并关闭，再次更新便可
> $ sudo apt update
> ```

### 安装ros2

ROS2提供了不同版本，主要区别是功能包数量不同。

```shell
### 
$ sudo apt install ros-foxy-desktop
###
$ sudo apt install ros-foxy-ros-base
```

### 安装自动补全工具

```shell
$ sudo apt install python3-argcomplete
```

### 设置环境变量

```shell
$ ls /opt/ros/
foxy/   noetic/                   # 这里noetic为ROS1，没有事先安装过就不会出现！
$ source /opt/ros/foxy/setup.bash
```

> **注意：**
>
> 如果有安装ros1则会有提示一个警告：
>
> ROS_DISTRO was set to 'noetic' before. Please make sure that the environment does not mix paths from different distributions.
>
> 这个时候需要把ROS1的环境变量去掉，增加ROS2的环境变量。
>
> 设置下`~/.bashrc`文件如下：
>
> ```shell
> #source /opt/ros/noetic/setup.bash
> source /opt/ros/foxy/setup.bash
> ```

### 安装编译工具colcon

```shell
$ sudo apt update && sudo apt install -y \
build-essential \
cmake \
git \
python3-colcon-common-extensions \
python3-pip \
python3-rosdep \
python3-vcstool \
wget
```

## 使用Docker安装ROS2

官方提供了ROS2镜像，可以去Docker镜像库中查看。

### 安装

参考：http://docs.ros.org/en/foxy/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html

```shell
## 下载镜像
$ docker pull osrf/ros:foxy-desktop

## 运行
$ docker run -it osrf/ros:foxy-desktop
```

### 运行

在两个独立容器内运行

```shell
### 打开第一个终端
$docker run -it --rm osrf/ros:foxy-desktop ros2 run demo_nodes_cpp talker
### 打开第二个终端
$ docker run -it --rm osrf/ros:foxy-desktop ros2 run demo_nodes_cpp listener
```

## 安装指定功能包

可以通过`apt-get install`命令安装ROS2提供的各个package，package对应的安装包格式为：

**ros2-<版本>-<package名称>**

例如：

```shell
sudo apt-get install ros-foxy-turtle-tf2-py ros-foxy-tf2-tools ros-foxy-tf-transformations
```

## VSCode

https://www.allisonthackston.com/articles/vscode-docker-ros2.html

### 智能提示

选择设置->Command Palette（快捷键ctrl+shift+P)，输入configuration，选择c/c++ Edit configuration，打开c_cpp_properties.json后，在`includePath`中添加ros的头文件路径即可！

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "/opt/ros/foxy/include",
                "/usr/include",
                "${workspaceFolder}/**"
            ],
            "defines": [],
            "compilerPath": "/opt/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-gcc",
            "cStandard": "gnu17",
            "cppStandard": "gnu++14",
            "intelliSenseMode": "linux-gcc-arm"
        }
    ],
    "version": 4
}
```

这种方式，对于远程连接开发也是有效的。

## 测试

- 打开一个终端，运行一个talker

```shell
$ ros2 run demo_nodes_cpp talker
[INFO] [1628004383.847069300] [talker]: Publishing: 'Hello World: 1'
[INFO] [1628004384.846837500] [talker]: Publishing: 'Hello World: 2'
```

如果要运行python版本的demo，将**demo_nodes_cpp**设置为**demo_nodes_py**即可。

- 再打开一个终端，运行一个listener

```shell
$ source /opt/ros/foxy/setup.bash
$ ros2 run demo_nodes_cpp listener
[INFO] [1628004397.878527800] [listener]: I heard: [Hello World: 15]
```

## 卸载

```shell
$sudo apt remove ros-foxy-* 
$sudo apt autoremove

### 或者
rm -rf ~/ros2_foxy    
```

这里是卸载foxy版本，如果安装的是其他版本，对应修改即可

## 问题和解决

### colcon build问题

1. setuptools相关

**问题：**

报错如下：

```shell
/home/alex/.local/lib/python3.6/site-packages/setuptools/dist.py:720: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  % (opt, underscore_opt)
  
Finished <<< pyServerClient [0.67s]

Summary: 2 packages finished [0.77s]
  2 packages had stderr output: pyServerClient py_pubsub
```

**解决：**

这个问题应该是setuptools安装的时候版本不匹配，可以卸载并安装最新的。

```shell
###卸载
$ python3 -m pip uninstall pip setuptools wheel
###重新安装
$ sudo apt-get --reinstall install python3-setuptools python3-wheel python3-pip
```

### 在conda中使用ros2

如果有的python安装包在conda环境下，例如在配置了一个跟踪检测的conda环境，希望在该环境下启动ros2能够调用次环境下的功能包，那么请在`~/.bahsrc`添加如下：

```shell
export PYTHONPATH=/home/alex/miniconda3/envs/alex/lib/python3.6/site-packages:$PYTHONPATH
```

