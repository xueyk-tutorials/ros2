
#include "parameter_basic/minimal_parameter.hpp"
using namespace std::chrono_literals;


MinimalParameter::MinimalParameter(std::string name) : Node(name)
{
    // 声明一个参数
    RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>> 声明参数");
    this->declare_parameter("name", "alex");
    this->declare_parameter("age", 32);

    // 声明一组参数，每组参数类型必须相同
    this->declare_parameters<std::string>("",
                                        std::map<std::string, std::string>{{"email", "alex@163.com"}, 
                                            {"address", "chengdu"}});
    this->declare_parameters<int16_t>("",
                                        std::map<std::string, int16_t>{{"number_a", 1}, 
                                            {"number_b", 2}});


    // 获取参数
    RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>> 获取参数");
    RCLCPP_INFO(this->get_logger(), "方式一：对成员变量赋值");
    this->get_parameter("name", this->_name);

    RCLCPP_INFO(this->get_logger(), "方式二: 获取Parameter类型返回值");
    rclcpp::Parameter p_age = this->get_parameter("age");
    RCLCPP_INFO(this->get_logger(), "parameter: type_name=%s, name=%s, value=%d", p_age.get_type_name().c_str(), p_age.get_name().c_str(), p_age.get_parameter_value().get<rclcpp::ParameterType::PARAMETER_INTEGER>());
    this->_age = p_age.as_int();
    this->_age = p_age.get_value<uint16_t>();
    RCLCPP_INFO(this->get_logger(), "age=%d", this->_age);



    //
    set_parameters_handle_ptr = this->add_on_set_parameters_callback(std::bind(&MinimalParameter::callback_on_set_parameters, this, std::placeholders::_1));
    //
    time_loop = this->create_wall_timer(1s, std::bind(&MinimalParameter::loop, this));
}



rcl_interfaces::msg::SetParametersResult MinimalParameter::callback_on_set_parameters(const std::vector<rclcpp::Parameter> &params)
{
    RCLCPP_INFO(this->get_logger(), "回调函数，对本节点进行参数设置，在外部调用了本节点成员函数:set_parameter()");
    RCLCPP_INFO(this->get_logger(), "参数数量一共有: %d", params.size());
    
    // RCLCPP_INFO(this->get_logger(), "查看参数信息");
    // for(auto param : params)
    // {
    //     RCLCPP_INFO(this->get_logger(), "parameter: type_name=%s, name=%s", 
    //                                     param.get_type_name().c_str(), 
    //                                     param.get_name().c_str());
    //     if(param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    //     {
    //         std::cout << param.get_value<std::string>() << std::endl;
    //     }
    //     else if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    //     {
    //         std::cout << param.get_value<int>() << std::endl;
    //     }
    // }
    // RCLCPP_INFO(this->get_logger(), "参数处理");
    for(auto param : params)
    {
        std::string email;
        uint16_t number_a;
        if(param.get_name() == "email")
        {   
            email = param.get_value<std::string>();
            // this->get_parameter("email", email);            // 这样获取的值，是this->declare_parameter()函数中的默认值
            RCLCPP_INFO(this->get_logger(), "email=%s", email.c_str());
        }
        if(param.get_name() == "number_a")
        {   
            number_a = param.get_value<int>();
            // this->get_parameter("number_a", number_a);      // 这样获取的值，是this->declare_parameter()函数中的默认值
            RCLCPP_INFO(this->get_logger(), "number_a=%d", number_a);
        }
    }

}

void MinimalParameter::loop()
{
    static int cnt = 0;
    // RCLCPP_INFO(this->get_logger(), "Looping...%d", cnt);
}