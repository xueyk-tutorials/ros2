#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace std::chrono_literals;

class Demo : public rclcpp::Node
{
public:
    Demo(std::string name="Demo") : Node(name)
    {
        rclcpp::QoS qos(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        sub_string = this->create_subscription<std_msgs::msg::String>("qos/string", 
                                                                        qos, 
                                                                        std::bind(&Demo::callback_string, this, std::placeholders::_1));        
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_string;
    // rclcpp::QoS qos;
    void callback_string(const std_msgs::msg::String::SharedPtr &msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data);
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