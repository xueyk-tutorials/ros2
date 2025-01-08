# ROS2开发者笔记——开机自启动



[How to start ROS2 node automatically after starting the system? - ROS Answers: Open Source Q&A Forum](https://answers.ros.org/question/333968/how-to-start-ros2-node-automatically-after-starting-the-system/)



## Ubuntu：自带开机启动

思路：通过ubuntu的`gnome-session-properties`功能启动。

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



## Ubuntu：通过systemctl启动ROS2

思路：Systemctl是linux系统继init.d之后的一个systemd工具，主要负责控制systemd系统和管理系统服务。systemd即为system daemon（系统守护进程）,是linux下的一种init软件。

这种方式不需要Ubuntu是desktop版本，支持Ubuntu18.04之后版本。

1. 添加服务

创建hello_ros2.service文件，内容如下：

```bash
[Unit]
Description=ros2 launch script
After=docker.target

[Service]
Type=simple

#指定用户名
User=helloworld
### 示例1: 运行命令行
ExecStart=/bin/bash -c "echo System start>>/home/helloworld/log.txt && export ROS_DOMAIN_ID=1 && export ROS_LOG_DIR=/home/helloworld/log && source /home/helloworld/install/setup.bash && bash /home/helloworld/install/alex/script/run.sh"

### 示例2: 运行脚本（确保脚本具备可执行权限）
ExecStart=/home/test.sh

ExecStop=/bin/bash -c "echo System stop>>/home/helloworld/log.txt"
PrivateTmp=true
KillMode=control-group


[Install]
WantedBy=multi-user.target
```

**将该文件拷贝至目录`/usr/lib/systemd/system/`！**

> 注意：
>
> - 其中路径名一定要使用绝对路径！！！
>
> - 尽量使用User关键字指定用户名，不然导致在终端输入`ros2 node list`无法打印正在运行的节点
>
> - ros2运行日志文件可能不会马上更新，需要服务启动后，等一会（一两分钟）才更新，也就是日志记录不是实时一行行写入，而是隔一会写一大块！
>
> - 如果是使用选项3，那么需要先创建运行脚本/home/test.sh！
>
>   赋权限`sudo chmod u+x /home/test.sh`
>
>   添加脚本内容，例如如下：
>
>   ```shell
>   #!/bin/bash
>   echo System start>>/home/helloworld/log.txt
>   export ROS_DOMAIN_ID=1
>   export ROS_LOG_DIR=/home/helloworld/log
>   source /home/helloworld/install/setup.bash
>   bash /home/helloworld/install/alex/script/run.sh
>   ```

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
# 实时显示程序打印的结果
$ sudo journalctl -u hello_ros2.service -f
# 或者，显示当前
$ systemctl status hello_ros2.service
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





## 开机启动docker容器进而自动启动ROS2

### 通过Dockerfile构建镜像

需要首先准备一个具备ROS2环境的docker镜像，然后使用Dockerfile基于该镜像再构建一个包含ROS2自启动命令的镜像。

在宿主机创建一个文件夹并在文件夹添加Dockerfile文件，内容如下：

```bash
FROM osrf_ros2:v1
COPY start_ros2.sh /home/
RUN echo "/opt/ros/foxy/setup.bash">>~/.bashrc \ 
    && echo "export ROS_DOMAIN_ID=0">>~/.bashrc
    
CMD ["/bin/bash", "/home/start_ros2.sh"]
```

> 这里osrf_ros2就是事先准备好的镜像。

在宿主机文件夹下创建start_ros2.sh并增加**运行权限**，添加内容如下：

```bash
#!/bin/bash

source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=0
ros2 run demo_nodes_cpp talker

LOG_FILE="/home/log.txt"
touch ${LOG_FILE}
echo "$(date)">>${LOG_FILE}
cnt=1
while [ ${cnt} -le 3 ]
do
    echo "$(date): ${cnt}">>${LOG_FILE}
    sleep 1s
    let cnt++
done
```

### 构建镜像

在Dockerfile文件所在目录下运行如下命令：

```bash
docker build -t drone_swarm .
# 如果添加版本标签的话如下，不添加默认是latest
docker build -t drone_swarm:v1 .
```

### 创建容器

通过如下命令创建容器：

```bash
docker run -d --net=host drone_swarm
```

如果容器启动后希望修改ROS2启动脚本内容，则可以在宿主机上通过`docker cp`命令将容器内的文件拷贝出来，修改后再拷贝回去。

```bash
# 从容器内拷贝出启动文件
docker cp drone_swarm:/home/start_ros2.sh .
# 对启动文件进行修改
vi start_ros2.sh
# 拷贝到容器
docker cp . drone_swarm:/home/start_ros2.sh
```

### 宿主机开机运行docker容器

创建一个启动脚本，通过systemctl等方式开机运行该启动脚本。

脚本如下：

```bash
#!/bin/bash

# 输入创建的镜像名称
container="drone_swarm"

log_file="/root/Desktop/log.txt"
touch $log_file
echo "[$(date "+%Y-%m-%d %H:%M:%S") start_docker.sh]: Try to start docker container">>$log_file

flag_success=false
# wait until docker start
cnt=1
while [ $cnt -le 5 ]
do
    echo "[$(date "+%Y-%m-%d %H:%M:%S") start_docker.sh]: Trying ${cnt}...">>$log_file
    echo $(docker version)>>$log_file
    is_running=$(docker version |grep "Server:" | awk '{print $1}')
    if [ -z "${is_running}" ]; then
        echo "[$(date "+%Y-%m-%d %H:%M:%S") start_docker.sh]: Docker not running">>$log_file
        echo $(docker version)>>$log_file
    else
        echo "[$(date "+%Y-%m-%d %H:%M:%S") start_docker.sh]: Docker already in running">>$log_file
        flag_success=true
        # start container
        docker start ${container}
        sleep 3s

        id=$(docker ps |grep "${container}" | awk '{print $1}')
        if [ -z "${id}" ]; then
            echo "[$(date "+%Y-%m-%d %H:%M:%S") start_docker.sh]: Start container failed">>$log_file
        else
            docker exec -it ${id} /bin/bash
            echo "[$(date "+%Y-%m-%d %H:%M:%S") start_docker.sh]: Start docker container success!">>$log_file
        fi
        break
    fi
    let cnt++
    sleep 1s
done

if [ "${flag_success}" = true ]; then
    echo "[$(date "+%Y-%m-%d %H:%M:%S") start_docker.sh]: Success">>$log_file
else
    echo "[$(date "+%Y-%m-%d %H:%M:%S") start_docker.sh]: Failed">>$log_file
fi
```

这个脚本首先会判断docker服务是否启动，服务启动后尝试运行容器，总计尝试多次直到成功。



## 开机启动dockers容器然后手动启动ROS2

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





