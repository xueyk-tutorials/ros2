#include <string>
#include <iostream>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "parameter_basic/minimal_parameter.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    std::cout << "node" << std::endl;

    rclcpp::init(argc, argv);    
    std::shared_ptr<MinimalParameter> node = std::make_shared<MinimalParameter>();


    //
    // RCLCPP_INFO(node->get_logger(), ">>>>>>>>>>>>>>>>> 对节点node进行设置参数，设置一个参数");
    // rclcpp::Parameter single_param("name", "AlexHAHA");
    // node->set_parameter(single_param);

    std::vector<rclcpp::Parameter> multi_params{};
    multi_params.emplace_back("email", "AlexHAHA@163.com");
    multi_params.emplace_back("address", "chengdu");
    multi_params.emplace_back("number_a", 11);
    multi_params.emplace_back("number_b", 12);
    RCLCPP_INFO(node->get_logger(), ">>>>>>>>>>>>>>>>> 对节点node进行设置参数，连续设置多个参数，相当于调用多次set_parameter");
    node->set_parameters(multi_params);

    rclcpp::WallRate rate(1s);
    rate.sleep();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}