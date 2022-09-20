#include <iostream>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std;
using namespace std::chrono_literals;

class Demo : public rclcpp::Node
{
public:
    Demo(std::string name="Demo") : Node(name)
    {
        pub_header = this->create_publisher<std_msgs::msg::Header>("demo/header", 10);
        // 定时器
        time_loop = this->create_wall_timer(1000ms, std::bind(&Demo::loop, this));
    }

private:
    rclcpp::TimerBase::SharedPtr time_loop;
    //
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_header;

    std_msgs::msg::Header header;
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    void loop()
    {
        // ROS2内置类型：builtin_interfaces/Time stamp
        header.stamp = clock->now();
        
        RCLCPP_INFO(this->get_logger(), "stamp sec=%ld, nanosec=%ld", header.stamp.sec, header.stamp.nanosec);
        pub_header->publish(header);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Demo> node = std::make_shared<Demo>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
