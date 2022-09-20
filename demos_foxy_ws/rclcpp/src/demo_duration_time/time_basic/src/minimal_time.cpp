/**
 * 
 */ 
#include <iostream>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "rcl/time.h"

#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;
using namespace std;


/**
 * rclcpp::Time构造函数测试
 */ 
void test01(rclcpp::Node::SharedPtr &node)
{
    /** 
     * 时间由两个部分组成，最终的时间为seconds+nanoseconds
     * Time (int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t clock_type=RCL_SYSTEM_TIME)
     */ 
    rclcpp::Time time1(100, 123, RCL_SYSTEM_TIME);
    RCLCPP_INFO(node->get_logger(), "构造函数1");
    RCLCPP_INFO(node->get_logger(), "time: sec=%f, nanosec=%ld", time1.seconds(), time1.nanoseconds());

    /**
     * 通过ROS2内置消息类型进行构造
     * Time (const builtin_interfaces::msg::Time &time_msg, rcl_clock_type_t clock_type=RCL_ROS_TIME)
     */ 
    builtin_interfaces::msg::Time stamp;
    stamp.sec     = 100;
    stamp.nanosec = 123;
    rclcpp::Time time2(stamp);
    RCLCPP_INFO(node->get_logger(), "构造函数2");
    RCLCPP_INFO(node->get_logger(), "time: sec=%f, nanosec=%ld", time2.seconds(), time2.nanoseconds());

    /**
     * 通过结构体rcl_time_point_t进行构造
     */ 
    rcl_time_point_t tp;
    tp.nanoseconds = 100123456789;
    tp.clock_type = RCL_SYSTEM_TIME;
    rclcpp::Time time3(tp);
    RCLCPP_INFO(node->get_logger(), "构造函数3");
    RCLCPP_INFO(node->get_logger(), "time: sec=%f, nanosec=%ld", time3.seconds(), time3.nanoseconds());
}

/**
 * 成员函数
 */ 
void test02(rclcpp::Node::SharedPtr &node)
{
    rclcpp::Time time = node->now();
    // 重载运算符，获取ROS2内置类型消息
    builtin_interfaces::msg::Time stamp = time;
    RCLCPP_INFO(node->get_logger(), "stamp.sec=%d, stamp.nanosec=%d", stamp.sec, stamp.nanosec);

    // 获取时间表示
    RCLCPP_INFO(node->get_logger(), "time: sec=%f, nanosec=%ld", time.seconds(), time.nanoseconds());

    // 获取时钟类型
    RCLCPP_INFO(node->get_logger(), "clock type=%d", time.get_clock_type());
}

/**
 * rclcpp::Time 与 std::chrono::time_point
 */ 
void test03(rclcpp::Node::SharedPtr &node)
{
    //Time->time_point
    RCLCPP_INFO(node->get_logger(), "convert rclcpp::Time to std::chrono::time_point");
    rclcpp::Time time = node->now();
    std::chrono::time_point<std::chrono::nanoseconds> tp(std::chrono::nanoseconds(time.nanoseconds()));
    RCLCPP_INFO(node->get_logger(), "time point: count=%ld", tp.time_since_epoch().count());

    //time_point->Time
    RCLCPP_INFO(node->get_logger(), "convert std::chrono::time_point to rclcpp::Time");
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp2 = std::chrono::system_clock::now();
    rclcpp::Time time2(rcl_time_point_t{tp2.time_since_epoch().count(), RCL_SYSTEM_TIME});
    RCLCPP_INFO(node->get_logger(), "time: sec=%f, nanosec=%ld", time2.seconds(), time2.nanoseconds());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("minimal_time");
    rclcpp::WallRate loop_rate(1);

    test01(node);
    test02(node);
    test03(node);
    //
    // while(rclcpp::ok())
    // {
    //     rclcpp::spin_some(node);
    //     loop_rate.sleep();
    // }
    rclcpp::shutdown();
    return 0;
}
