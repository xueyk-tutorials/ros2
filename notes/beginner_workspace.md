# ROS2开发者笔记——创建工程

## ROS2目录结构

工作空间主要包含的文件夹有：`src/`, `install/`,  `build/`, `log/`。

文件结构如下：

```shell
|--workspace_folder/
    |--src/
      |--package_cpp/
          |--src/
          |--CMakeLists.txt
          |--package.xml
      |--package_py/                     # 用户建立python包目录结构示例
      	  |--package_py/
      	      |--node1.py
      	      |--node2.py
          |--resource/package_cpp
          |--test
              |--test_copyright.py
              |--test_flake8.py
              |--test_pep257.py
          |--setup.py
          |--setup.cfg
          |--package.xml                  #作者信息、功能包描述、指明需要依赖的其他包
	|-- install/
	  |-- package1_py/
	      |-- lib/
	          |-- package1_py/
	              |-- node1               # 编译生成的对应节点的可执行二进制文件
	          |-- python3.6/package1_py/
	              |-- node1.py            # 编译后拷贝过来的节点程序文件，会有节点可执行二进制文件调用
	          
```

## 创建工作空间

### 建立工程目录

工程目录是整个ROS2工程的根目录，通常命名为**xxx_ws**，且根目录一定有**src/**文件夹用于放置功能包。

```shell
$ mkdir -p alex_ros2_ws/src
$ cd alex_ros2_ws/src/
```

### 安装编译工具calcon

```shell
sudo apt install python3-colcon-common-extensions
#或者
sudo apt update && sudo apt install -y \
build-essential \
cmake \
git \
python3-colcon-common-extensions \
python3-pip \
python-rosdep \
python3-vcstool \
wget
```

### 创建功能包

ROS2所有的功能包都是在工程目录的src/文件夹下。功能包创建使用`ros2 pkg create`命令。

使用`--build-type`指定功能包类型，创建的功能包类型有：

- ament_cmake：c++功能包
- ament_python：python功能包

```shell
$ ros2 pkg create --build-type ament_cmake <pkg_name>
```

### 编译

进入ros2工程目录下，然后编译：

```shell
$ colcon build

Summary: 0 packages finished [0.15s]
```

colcon build后边还可以跟一些常用的参数：  

1. --packages-up-to ：编译指定的功能包，而不是整个工作空间
2. --symlink-install ：节省每次重建python脚本的时间
3. --packages-select： 选择要编译的package
4. --event-handlers console_direct+ ：在终端中显示编译过程中的详细日志



## 安装工程依赖

检查工程相关的依赖是否齐全并且安装缺少的packages。

```shell
$ rosdep install -i --from-path src --rosdistro foxy -y
```

