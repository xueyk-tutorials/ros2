#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std;
using namespace std::chrono_literals;


class SimpleService : public rclcpp::Node
{
public:
    SimpleService(std::string name="SimpleService") : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "SimpleService");
        srv_add_two_ints = this->create_service<example_interfaces::srv::AddTwoInts>("simple/add_two_ints",
                                                                                    std::bind(&SimpleService::handler_add_two_ints,
                                                                                    this,
                                                                                    std::placeholders::_1,
                                                                                    std::placeholders::_2));
    }

    void handler_add_two_ints(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                              const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        rclcpp::WallRate rate(1000ms);
        //
        RCLCPP_INFO(this->get_logger(), "%d + %d:", request->a, request->b);
        response->sum = request->a + request->b;
    }

private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_add_two_ints;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<SimpleService> node = std::make_shared<SimpleService>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}