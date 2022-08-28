#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

/**
 * 
 */ 
bool use_async = true;

class AsyncService : public rclcpp::Node
{
public:
    AsyncService(std::string name="AsyncService") : Node(name)
    {
        // 回调组
        callback_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

        //
        service = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                                                                            std::bind(&AsyncService::add, this, std::placeholders::_1, std::placeholders::_2),
                                                                            rmw_qos_profile_services_default,
                                                                            callback_group);
        timer_ = this->create_wall_timer(1000ms, std::bind(&AsyncService::loop, this));
    }

private:
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service;

    void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), ">>>>>>Service handler");
        RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld" " b: %ld",
                        request->a, request->b);

        // 模拟在服务响应中增加一个较为耗时的工作，查看是否对节点其他工作有影响
        rclcpp::Rate rate(1000ms); 
        for(int i=0; i<3; ++i)
        {
            RCLCPP_INFO(this->get_logger(), "i=%d", i);
            rate.sleep();
        }
        //
        RCLCPP_INFO(this->get_logger(), "=====sending back response: [%ld]", (long int)response->sum);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int count_;

    void loop()
    {
        RCLCPP_INFO(this->get_logger(), "loop: count=%d", count_++);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // AsyncService::SharedPtr node = AsyncService::make_shared();
    std::shared_ptr<AsyncService> node = std::make_shared<AsyncService>();

    if (use_async)
    {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    else
    {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
}