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



