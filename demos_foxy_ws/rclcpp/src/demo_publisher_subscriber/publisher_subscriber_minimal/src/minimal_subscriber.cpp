
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber(std::string name="MinimalSubscriber") : Node(name)
    {
        /**
         * 订阅消息处理方式一：通过注册回调函数
         * 这种方式逻辑清晰易读，但需要声明并实现回调函数
         */ 
        // subscriber_ = this->create_subscription<std_msgs::msg::String>("topic", 
        //                                                                 10, 
        //                                                                 std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
        /**
         * 订阅消息处理方式二：通过注册lambda函数
         * 这种方式简洁，适合做简单处理
         */ 
        subscriber_ = this->create_subscription<std_msgs::msg::String>("topic",
                                                                        10,
                                                                        [this](const std_msgs::msg::String::SharedPtr msg){
                                                                            this->sub_message_ = msg;
                                                                            RCLCPP_INFO(this->get_logger(), "方式二：subscriber: %s", msg->data.c_str());
                                                                        });
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "方式一：subscriber: %s", msg->data.c_str());
    }

    std_msgs::msg::String::SharedPtr sub_message_;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MinimalSubscriber> node = std::make_shared<MinimalSubscriber>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}