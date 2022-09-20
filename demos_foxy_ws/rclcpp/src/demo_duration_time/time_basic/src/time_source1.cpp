#include <iostream>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"

using namespace std::chrono_literals;
using namespace std;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("time_source");
    rclcpp::WallRate loop_rate(1);

    rclcpp::Time time_ros;
    
    /**
     * TimeSource只能绑定RCL_ROS_TIME
     */
    rclcpp::TimeSource ts_ros_time(node);
    rclcpp::Clock::SharedPtr clock_ros = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts_ros_time.attachClock(clock_ros);

    //
    while(rclcpp::ok())
    {
        time_ros = clock_ros->now();
        RCLCPP_INFO(node->get_logger(), "ros_time:             sec=%f, nanosec=%ld", time_ros.nanoseconds(), time_ros.seconds());

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
