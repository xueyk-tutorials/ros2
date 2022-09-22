#include <iostream>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

using namespace std;
using namespace std::chrono_literals;

void loop()
{
    static int cnt = 0;
    cout << "loop, cnt" << cnt++ << endl;
}
/**
 * @description: 构造函数测试1
 * Node有两种构造函数：
 * 第一种）rclcpp::Node::Node(const std::string & node_name, const NodeOptions & options = NodeOptions())
 * 第二种）rclcpp::Node::Node(const std::string & node_name,const std::string & namespace_, const NodeOptions & options = NodeOptions())	
 * @param {int} argc
 * @param {char} *argv
 * @return {*}
 * @author: xueyuankui
 */
void test01(int argc, char *argv[])
{
    cout << "test01" << endl;
    // 构造方法一：最简单的构造方法
    // rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test01_node");

    // 构造方法二：指定NodeOption，用来设置是否进程间通信、参数发布订阅、参数QoS等
    rclcpp::NodeOptions option = rclcpp::NodeOptions();
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test01_node", option);
    
    // 构造方法三：增加namespace参数，这样节点运行后其名字前会被限定一个命名空间，即/namespace_/node_name
    // rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test01_node", "node_ns");          // ros2 node list: /node_ns/test01_node


    rclcpp::spin(node);
}

void test02(int argc, char *argv[])
{
    
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    if(argc < 2)
    {
        cout << "Help: " << "ros2 run <node> <test01|test02>" << endl;
    }
    else
    {
        std::string fun_name(argv[1]);
        if(fun_name == "test01")
        {
            test01(argc, argv);
        }
        else if(fun_name == "test02")
        {
            test02(argc, argv);
        }
    }
    
    rclcpp::shutdown();

    return 0;
}