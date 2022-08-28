# ROS2开发者笔记——打包安装与发布

## C++



### 示例

#### launch启动自动加载配置文件

很多时候我们的节点运行需要加载一些配置文件，例如目标跟踪节点，需要加载模型参数文件。我们需要知道这类配置文件所在绝对路径，如果这个绝对路径作为变量参数写在程序中，那么当工程在不同电脑之间移植时需要频繁改变代码。

为了解决这个问题，我们可以进行如下操作：

- 将节点配置文件安装至install/路径下
- 在launch文件中，获取节点所在package的绝对路径，然后再通过相对路径，查找到配置文件。

**package目录结构如下:**

我们希望目标跟踪节点能够运行时加载`models/`下的.onnx模型。

```shell
.
├── CMakeLists.txt
├── include
│   ├── modules
│   │   └── objtracker_node.hpp
│   ├── stark
│   │   ├── stark.h
│   │   └── utils.h
├── launch
│   └── modules.launch.py
├── package.xml
└── src
    ├── objtracker_main.cpp
    ├── objtracker_node.cpp
    ├── stark
    │   ├── models
    │   │   ├── backbone_bottleneck_pe.onnx
    │   │   └── complete.onnx
    │   ├── stark.cpp
```

**安装**

在package下的CMakeLists中，将需要的配置文件打包安装

```CMake
# Install models
install(DIRECTORY src/stark/models/
  DESTINATION share/${PROJECT_NAME}/models/
)
```

将package编译后，就会将models拷贝至安装路径下：

```shell
.
├── install
│   ├── modules
    |	├── include/
    |	├── lib/
    |	├── share/
    		|── modules/
    			|── models/   #安装完成
```

**launch文件**

在launch文件中，通过`get_package_share_directory`函数找到package的绝对路径，然后再查找models的相对路径，然后通过ROS参数的方式，将路径作为参数传递给节点。

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取路径
    print(get_package_share_directory("modules"))
    config_tracker_onnx_path = os.path.join(
        get_package_share_directory("modules"),
        "models/backbone_bottleneck_pe.onnx"
    )
    # 将路径作为参数传给节点
	return LaunchDescription([
        Node(package='modules',
             node_executable='objtracker_main',
             node_name='objtracker_main',
             output='screen',
             parameters=[{'tracker_model_path1':config_tracker_onnx_path},
             ])
    ])
```

**节点**

```c++
this->declare_parameter<std::string>("tracker_model_path1", "");

this->get_parameter("tracker_model_path1",  param_tracker_model_path1);
```



## python

### setup.py

setup.py是python第三方库setuptools提供的一种打包和安装的规则，这个文件类似于CMakeLists.txt，能够将开发代码进行统一的打包和安装。

一个初始状态的setup.py如下：

```python
from setuptools import setup
package_name = 'learn_pkg'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='cetcs_jizhi_group@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

```

> 这个是从一个名称为**learn_pkg**的ros2包中的setup.py文件拷贝而来！

- **packages**

用于进行package的打包。

例如有个package如下：

```python
|-- src/learn_pkg/learn_pkg/
    |-- Drivers/
        |-- __init__.py
        |-- pod.py
        |-- comm.py
        |-- drone.py
```

那么打包设置为：

```python
packages = [package_name, 'Drivers'],
```

**如何在其他文件中引用Drivers呢？**

通过package的设置后，ROS2进行工程编译，会把Drivers打包到新生成的install目录下，并且将这个目录下的`install/learn_pkg/lib/python3.6/site-packages`添加到python环境变量中，就相当于这个Drivers安装在了本地计算机！其他python文件进行引用时使用：

```shell
import learn_pkg.Drivers.pod as pod
```

- **data_files**

用于进行依赖文件的打包，例如配置文件、图片等等。

data_files是个元组列表，其中的元组内有两个元素。

例如ros包目录如下：

```shell
|-- src/learn_pkg/learn_pkg/
    |-- Drivers/
    |-- config.txt
    |-- my_node.py
```

**如果希望把`config.txt`作为配置文件打包到安装目录，可设置如下:**

```python
import os
data_files=[
        (os.path.join('lib/python3.6/site-packages', package_name), package_name + 'config.txt'),
    ],
```

这时对工程进行编译后，生成的install目录如下：

```python
|-- ROS2_WS/
	|-- install/
		|-- learn_pkg/
			|-- libs/
				|-- learn_pkg/
				|-- python3.6/site-packages/
					|-- learn_pkg/
                    	|-- my_node.py
                        |-- config.txt
```

**那如何在启动my_node.py节点时加载config.txt呢？**

由于ros2启动节点时，就是运行的安装目录下的python文件`ROS2_WS/install/learn_pkg/lib/python3.6/site-packages/learn_pkg/my_node.py`，而且配置文件与这个python文件的相对路径是明确的，那么我们可以通过获取python文件的绝对路径后，再通过相对路径找到配置文件。例如my_node.py可以添加如下路径相关的操作代码：

```python
# 通过__file__获取运行的python文件。
path = os.path.join(os.path.dirname(__file__), ‘config.txt’)
```



### 示例

#### 在节点中引用文件

如果某个功能涉及到的功能较多，需要多个文件才能完成功能实现，则这些文件夹需要放到一个文件夹下，然而在节点中又需要导入这些功能文件，这时直接使用import是无效的，因为ros2节点运行时的可执行文件是在`install`文件夹下的，无法找到这些功能文件。

例如目录结构（未编译之前）如下：

```shell
|-- ROS2_WS/
	|-- src/
		|-- learn_pkg/
			|-- learn_pkg/
				|-- my_libs/
					|-- functions.py
				|-- my_node.py
			|-- setup.py
```

节点程序`my_node.py`如下：

```python
import rclpy
from rclpy.node import Node
### 导入库中的所有功能包
from my_libs import *

def main(args=None):
    print("INFO: lib_caller.py")
    rclpy.init(args=args)
    node = Node('lib_caller')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

这时候就需要`setup.py`进行相应配置。

```python
from setuptools import setup
import os
from glob import glob

package_name = 'learn_pkg'

setup(
data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', *.launch.py'))),
        ######################
        ### 将my_libs库中的所有文件拷贝至安装目录下
        ######################
        (os.path.join('lib', package_name + '/my_libs'), glob(package_name + '/my_libs/*.py')),
    ],
)
```

编译后的目录结构如下：

```shell
|-- ROS2_WS/
	|-- install/
		|-- learn_pkg/
			|-- lib/
				|-- my_libs/      # 拷贝过来的库
				|-- my_node       # 编译后的执行文件
			|-- share/
	|-- src/
		|-- learn_pkg/
			|-- learn_pkg/
			|-- setup.py
```

