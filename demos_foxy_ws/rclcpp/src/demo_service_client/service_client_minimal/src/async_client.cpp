#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class AsyncClient : public rclcpp::Node
{
public:
    AsyncClient(std::string name="AsyncClient") : Node(name)
    {
        client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints", rmw_qos_profile_services_default);
        // 创建定时器，定时器初始化后开始计时，到达规定时间便触发回调函数，然后重新计时！故第一次触发需要等到达规定时间才执行，而不是一创建就立马触发第一次。
        timer_ = this->create_wall_timer(5000ms, std::bind(&AsyncClient::loop, this));
    }
private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client;

    // 回调函数：用于服务响应处理
    void response_add_two_ints(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future)
    {
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response = result_future.get();
        RCLCPP_INFO(this->get_logger(), "=====sum=%d", response->sum);
    }
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 定时器函数，请求服务
    void loop()
    {
        RCLCPP_INFO(this->get_logger(), ">>>>>>Client request");
        RCLCPP_INFO(this->get_logger(), "[%d] + [%d] = ?", a, b);
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request= std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = this->a++;
        request->b = this->b++;

        // 请求服务并注册请求结果响应的回调函数（用于处理服务反馈结果）
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future = client->async_send_request(request, 
                                                                                                                    std::bind(&AsyncClient::response_add_two_ints, this, std::placeholders::_1));
    }
    int a = 0;
    int b = 0;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // AsyncClient::SharedPtr node = AsyncClient::make_shared();
    std::shared_ptr<AsyncClient> node = std::make_shared<AsyncClient>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}