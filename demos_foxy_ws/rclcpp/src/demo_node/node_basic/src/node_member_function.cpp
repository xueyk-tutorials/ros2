#include <stdio.h>
#include <iostream>
#include <string>
#include <chrono>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace std::chrono_literals;

/**
 * @description: 成员函数测试1
 * @param {int} argc
 * @param {char} *argv
 * @return {*}
 * @author: xueyuankui
 */
void test01(int argc, char *argv[])
{
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    rclcpp::NodeOptions options;

    auto node1 = std::make_shared<rclcpp::Node>("node1", "node_ns", options);
    exec.add_node(node1);

    auto node2 = std::make_shared<rclcpp::Node>("node2", "node_ns", options);
    exec.add_node(node2);

    auto node3 = std::make_shared<rclcpp::Node>("node3", options);
    exec.add_node(node3);

    /** 
     * Node成员函数测试：get_name()、get_namespace()、get_fully_qualified_name()
     */ 
    cout << "node->get_name(): " << node1->get_name() << endl;
    cout << "node->get_namespace(): " << node1->get_namespace() << endl;
    cout << "node->get_fully_qualified_name(): " << node1->get_fully_qualified_name() << endl;

    /**
     * Node成员函数测试: get_node_names
     * 可以获取所有活动的node，包括获取其他终端运行的node，不论这些节点所属的namespace是否相同
     */ 
    // 这个延迟很重要，因为发现其他终端的节点需要一定时间。
    rclcpp::Rate rate(1000ms);
    rate.sleep();

    cout << "node->get_node_names()" << endl;
    std::vector<std::string> node_names1 = node1->get_node_names();
    cout << "number of node:" << node_names1.size() << endl;
    for(auto n : node_names1)
    {
        cout << "n=" << n << endl;
    }
    std::vector<std::string> node_names2 = node2->get_node_names();
    cout << "number of node:" << node_names2.size() << endl;    
    for(auto n : node_names2)
    {
        cout << "n=" << n << endl;
    }
    std::vector<std::string> node_names3 = node3->get_node_names();
    cout << "number of node:" << node_names3.size() << endl;
    for(auto n : node_names3)
    {
        cout << "n=" << n << endl;
    }

    exec.spin();
}
void test02(int argc, char *argv[])
{
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test02_node");
    auto pub_str1_1 = node->create_publisher<std_msgs::msg::String>("pub_str1", 1);
    auto pub_str1_2 = node->create_publisher<std_msgs::msg::String>("pub_str1", 1);
    auto pub_str2 = node->create_publisher<std_msgs::msg::String>("pub_str2", 1);

    /** 
     * Node成员函数测试：count_publishers
     */ 
    cout << ">>>>>>>>>>>>>>> count_publishers" << endl;
    std::size_t num_pubs1 = node->count_publishers("pub_str1");
    cout << "topic 'pub_str1' publisher numbers=" << num_pubs1 << endl;
    std::size_t num_pubs2 = node->count_publishers("pub_str2");
    cout << "topic 'pub_str2' publisher numbers=" << num_pubs2 << endl;

    /**
     * Node成员函数测试：get_topic_names_and_types
     */ 
    cout << ">>>>>>>>>>>>>>> get_topic_names_and_types" << endl;
    std::map<std::string, std::vector<std::string>> res = node->get_topic_names_and_types();
    for(std::map<std::string, std::vector<std::string>>::iterator it=res.begin(); it!=res.end(); ++it)
    {
        cout << "------------" << endl;
        cout << "name=" << it->first << endl;
        cout << "type=" << endl;
        for(auto s : it->second)
        {
            cout << s << " ";
        }
        cout << endl;
    }

    /**
     * Node成员函数测试：get_publishers_info_by_topic
     */
    cout << ">>>>>>>>>>>>>>> get_publishers_info_by_topic" << endl;
    std::vector<rclcpp::TopicEndpointInfo> topic_infos = node->get_publishers_info_by_topic("pub_str1");
    for(auto info : topic_infos)
    {   
        cout << "------------" << endl;
        cout << "topic_type: " << info.topic_type() << endl;
        cout << "node_name: " << info.node_name() << endl;
        std::array<uint8_t, 24U> gids = info.endpoint_gid();
        cout << "endpoint_gid: size is " << gids.size() << endl;
        for(auto gid : gids)
        {
            // cout << gid << " ";
            printf("%d ", gid);
        }
        cout << endl;
    }
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