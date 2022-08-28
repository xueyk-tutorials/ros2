#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"

using namespace std;
using namespace std::chrono_literals;


void test01(const rclcpp::Node::SharedPtr &node)
{
    rclcpp::Clock clock_system(RCL_SYSTEM_TIME);
    rclcpp::Clock clock_steady(RCL_STEADY_TIME);

    rclcpp::Time time_system = clock_system.now();
    rclcpp::Time time_steady = clock_steady.now();
    RCLCPP_INFO(node->get_logger(), "system time: nanosecond=%ld", time_system.nanoseconds());
    RCLCPP_INFO(node->get_logger(), "steady time: nanosecond=%ld", time_steady.nanoseconds());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("minimal_clock");
    test01(node);

    rclcpp::shutdown();

    return 0;
}