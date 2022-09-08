#include <string>
#include <map>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class MinimalParameter : public rclcpp::Node
{
public:
    MinimalParameter(std::string name="MinimalParameter");
    
private:
    std::string _name;
    uint16_t _age;
    //
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr set_parameters_handle_ptr;
    rcl_interfaces::msg::SetParametersResult callback_on_set_parameters(const std::vector<rclcpp::Parameter> &params);
    //
    rclcpp::TimerBase::SharedPtr time_loop;
    void loop();
};