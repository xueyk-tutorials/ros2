
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher(std::string name="MinimalPublisher") : Node(name)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback()
    {
        std_msgs::msg::String message;
        message.data = "Hello, world!" + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
        publisher_->publish(message);
    }
    size_t count_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MinimalPublisher> node = std::make_shared<MinimalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}