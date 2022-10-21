/*
 * @Description: 对创建的插件进行测试，在任意package中都可以使用插件！只不过这里放到了polygon_base package中进行测试而已。
 * @Version: 1.0
 * @Author: xueyuankui
 * @Date: 2022-09-14 22:48:25
 * @LastEditors: xueyuankui
 * @LastEditTime: 2022-09-23 10:27:43
 */
#include <iostream>
#include <pluginlib/class_loader.hpp>
#include <polygon_base/regular_polygon.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");

  try
  {
    for(auto &name : poly_loader.getDeclaredClasses())
    {
      std::cout << "name=" << name << std::endl;
    }
    std::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle");
    triangle->initialize(10.0);

    std::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createSharedInstance("polygon_plugins::Square");
    square->initialize(10.0);

    printf("Triangle area: %.2f\n", triangle->area());
    printf("Square area: %.2f\n", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}