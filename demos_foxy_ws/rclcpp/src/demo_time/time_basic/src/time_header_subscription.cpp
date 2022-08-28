#include <iostream>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;
using namespace std;

class Demo : public rclcpp::Node
{
public:
    Demo(std::string name="Demo") : Node(name)
    {
        sub_header = this->create_subscription<std_msgs::msg::Header>("demo/header", 
                                                                    10, 
                                                                    std::bind(&Demo::callback_header, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_header;

    rclcpp::Clock::SharedPtr clock = this->get_clock();
    void callback_header(const std_msgs::msg::Header::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "stamp sec=%ld, nanosec=%ld", msg->stamp.sec, msg->stamp.nanosec);
        //
        rclcpp::Time t_header(msg->stamp);
        rclcpp::Time t_now = clock->now();
        RCLCPP_INFO(this->get_logger(), "time msg=%f", t_header.seconds());
        RCLCPP_INFO(this->get_logger(), "time now=%f", t_now.seconds());
        rclcpp::Duration d = t_now - t_header;
        RCLCPP_INFO(this->get_logger(), "duration=%f", d.seconds());
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
