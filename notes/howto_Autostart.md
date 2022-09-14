# ROS2开发者笔记——开机自启动



[How to start ROS2 node automatically after starting the system? - ROS Answers: Open Source Q&A Forum](https://answers.ros.org/question/333968/how-to-start-ros2-node-automatically-after-starting-the-system/)

## 方法综述



### 方法1：ubuntu自带开机启动（success）

思路：通过ubuntu的`gnome-session-properties`功能启动

该方法需要ubuntu为Desktop版本！

1. 编辑~/.bashrc

   在环境变量脚本中添加新建终端

   ```shell
   . $HOME/Desktop/ros_ws/install/setup.bash
   cd $HOME/Desktop/ros_ws
   gnome-terminal -x rosrun turtlesim turtlesim_node
   gnome-terminal -x rosrun turtlesim turtle_teleop_key 
   ```

   

2. 配置开启选项管理

   命令终端输入`gnome-session-properties`打开Ubuntu开机首选项管理，然后添加一个启动程序，**名称**根据情况自行填写，**命令**输入gnome-terminal。

3. 重启计算机

   重启后，等待桌面显示，可看的会自动弹出3个终端，后面两个终端依次运行ros2相关节点！



### 方法2：通过python文件(fail)

思路：首先通过ubuntu的service添加开机自启动shell脚本，该shell脚本会运行一个python程序，python启动一个网络监听线程，等待接收控制指令，当接收到其他计算机发送的启动指令后，通过os.system()启动shell命令！



## 开机启动docker

本节要实现的功能是开机启动docker容器，并在docker容器中运行ros！

我们首先熟悉下docker相关操作以及启动docker如何运行ros，然后再讲解如何开机启动docker！

### 启动容器后运行ROS2

下载的ROS2 docker镜像，一般都会提供ROS入口脚本（容器根目录下的`ros_entrypoint.sh`），该脚本是容器启动的默认脚本，用于添加ROS2环境变量，一般内容如下：

```shell
#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
exec "$@"
```

#### 运行单个节点

创建好ROS2容器，可以直接输入运行talker节点的命令：

```shell
$ docker run -it osrf/ros:foxy-desktop ros2 run demo_nodes_cpp talker
```



#### 运行launch（方法一）

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

#### 运行launch（方法二）

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

#### 运行launch（方法三）

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

### 通过systemctl启动

Systemctl是linux系统继init.d之后的一个systemd工具，主要负责控制systemd系统和管理系统服务。systemd即为system daemon（系统守护进程）,是linux下的一种init软件。

1. 添加服务

创建hello_ros2.service文件，内容如下：

```bash
[Unit]
Description=ros2 launch script
After=docker.target

[Service]
Type=simple

### 示例1: 运行命令行
ExecStart=/bin/bash -c "export ROS_DOMAIN_ID=50 && && source /home/helloworld/install/setup.bash && bash /home/helloworld/install/alex/script/run.sh"

### 示例2: 运行命令行，启动docker容器并运行ros入口脚本
ExecStart=/bin/bash -c " echo hello >> /home/project_tc/log.txt && docker start frosty_tu && docker exec -it frosty_tu /ros_entrypoint.sh"

### 示例3: 运行脚本（确保脚本具备可执行权限）
ExecStart=/home/test.sh

PrivateTmp=true
KillMode=control-group


[Install]
WantedBy=multi-user.target
```

**将该文件拷贝至目录`/usr/lib/systemd/system/`！**

如果是使用选项2，那么需要先创建运行脚本！

2. 服务控制

- 重新加载

```shell
sudo systemctl daemon-reload
```

- 启动服务

```shell
systemctl start hello_ros2.service
```

- 查看服务结果

```shell
$ sudo journalctl -u hello_ros2.service -f
```

- 开机自启动

```shell
systemctl enable hello_ros2.service
```

其他命令

```shell
#停止服务
systemctl stop hello_ros2.service
#查看已启动的服务列表
systemctl list-unit-files|grep enabled
#显示所有已启动的服务
systemctl list-units --type=service
```

