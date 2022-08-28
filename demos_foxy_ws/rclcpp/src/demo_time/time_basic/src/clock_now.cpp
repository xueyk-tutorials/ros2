#include <iostream>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"

using namespace std::chrono_literals;
using namespace std;

class DemoTimeSource : public rclcpp::Node
{
public:
    DemoTimeSource(std::string name="DemoTimeSource") : Node(name)
    {
        clock_steady = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
        // 定时器
        time_loop = this->create_wall_timer(1000ms, std::bind(&DemoTimeSource::loop, this));

    }

private:
    rclcpp::TimerBase::SharedPtr time_loop;
    //
    rclcpp::Time system_time, steady_time;
    rclcpp::Clock::SharedPtr clock_steady;

    void loop()
    {
        /**
         * chrono
         */
        RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> chrono");
        auto tp_system = std::chrono::system_clock::now(); 
        time_t tt  = std::chrono::system_clock::to_time_t(tp_system);
        auto d_system_ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp_system.time_since_epoch());
        RCLCPP_INFO(this->get_logger(), "system_clock:             sec=%ld, ms=%ld", tt, d_system_ms.count());

        std::chrono::steady_clock::time_point tp_steady  = std::chrono::steady_clock::now(); 
        std::chrono::steady_clock::duration   dtn        = tp_steady.time_since_epoch();
        double secs = dtn.count() * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
        auto d_steady_ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp_steady.time_since_epoch());
        RCLCPP_INFO(this->get_logger(), "steady_clock              sec:%lf, ms=%ld", secs, d_steady_ms.count());
        
        /**
         * 默认时钟RCL_SYSTEM_TIME
         */
        RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> RCL_SYSTEM_TIME");
        // 获取时间
        RCLCPP_INFO(this->get_logger(), "clock type=%d", rclcpp::Clock().get_clock_type());
        system_time = rclcpp::Clock().now();
        // 打印Epoch时间戳，分别使用秒和纳秒为单位表示。
        RCLCPP_INFO(this->get_logger(), "system_time:              sec=%f, nanosec=%ld", system_time.seconds(), system_time.nanoseconds());

        /**
         * RCL_STEADY_TIME
         */
        RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> RCL_STEADY_TIME");
        RCLCPP_INFO(this->get_logger(), "clock type=%d", clock_steady->get_clock_type());
        steady_time = clock_steady->now();
        RCLCPP_INFO(this->get_logger(), "steady_time:              sec=%f, nanosec=%ld", steady_time.seconds(), steady_time.nanoseconds());

        /**
         * this->get_clock()获取的是RCL_ROS_TIME
         * this->get_clock()->now()
         * this->now()
         */
        RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> this->get_clock()->now(), this->now()");
        rclcpp::Clock::SharedPtr clock_node = this->get_clock();
        RCLCPP_INFO(this->get_logger(), "clock type=%d", clock_node->get_clock_type());

        rclcpp::Time t2 = this->get_clock()->now();
        rclcpp::Time t3 = this->now();
        RCLCPP_INFO(this->get_logger(), "this->get_clock()->now(): sec=%f, nanosec=%ld", t2.seconds(), t2.nanoseconds());
        RCLCPP_INFO(this->get_logger(), "this->now():              sec=%f, nanosec=%ld", t3.seconds(), t3.nanoseconds());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<DemoTimeSource> node = std::make_shared<DemoTimeSource>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
