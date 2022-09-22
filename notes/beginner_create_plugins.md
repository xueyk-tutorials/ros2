# ROS2插件

## 参考

http://docs.ros.org/en/ros2_packages/humble/api/pluginlib/

## 插件思想

插件（plugin）的核心包括：良好的隔离性、优秀的扩展性、动态加载。

从实现上来说是通过类继承实现的。

为了解释插件的原理，我们定义三个package：

- pkg_polygon_base：定义插件基类。

- pkg_polygon_plugins：用户自定义插件，进行扩展。
- pkg_robot：用户工程，使用插件！

### 动态加载

插件的动态加载从C++编程规则上来说，就是提供了一种不需要通过include插件子类头文件的方式而是使用第三方工具（pluginlib）来引入插件子类（只需要让pluginlib知道插件名字和所在package即可）。

我们假设在包pkg_polygon_plugins中定义了两个多边形插件（nav的子类）：Triangle，Square。在包pkg_robot中需要使用导航插件时，并不是通过c++编程规则（如include，using namespace）导入Triangle，Square，而是使用pluginlib提供的ClassLoader类来加载插件。

- 实例化ClassLoader

```c++
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");
```

- 导入插件

  我们只需要在调用`createSharedInstance`方法时指定一个字符串（由插件package名字和插件名组成）参数即可加载插件子类！

```c++
  try
  {
    std::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle");
    triangle->initialize(10.0);

    printf("Triangle area: %.2f\n", triangle->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }
```

由于字符串参数是可以很方便的修改，例如在程序运行时，通过一些通信方法（ros消息、tcp/udp通信）指定字符串，就可以导入插件了！这样更换插件也非常方便！