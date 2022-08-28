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
        timer_loop = this->create_wall_timer(1000ms, std::bind(&Demo::loop, this));
        rclcpp::QoS qos(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        printf_qos(qos);
        // 设置qos
        qos.durability_volatile();                           // 不保留样本
        qos.best_effort();
        qos.deadline(std::chrono::milliseconds(30ms));
        qos.lifespan(std::chrono::milliseconds(20ms));

        printf_qos(qos);

        pub_string = this->create_publisher<std_msgs::msg::String>("qos/string", qos);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_loop;

    // rclcpp::QoS qos(KeepLast(1), rmw_qos_profile_sensor_data);
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_string;

    void loop()
    {
        auto time_now = this->now();
        RCLCPP_INFO(this->get_logger(), "sec=%f, nanosec=%ld", time_now.seconds(), time_now.nanoseconds());
        std_msgs::msg::String msg;
        msg.data = "hello";
        pub_string->publish(msg);
    }
    void printf_qos(rclcpp::QoS &qos)
    {
        // 查看QOS参数
        // 获取mw_qos_profile_t
        // rclcpp::rmw_qos_profile_t qos_profile = qos.get_rmw_qos_profile();
        auto qos_profile = qos.get_rmw_qos_profile();
        rmw_time_t       rmw_time_deadline = qos_profile.deadline;
        rmw_time_t       rmw_time_lifespan = qos_profile.lifespan;
        rmw_time_t       rmw_time_liveliness = qos_profile.liveliness_lease_duration;

        RCLCPP_INFO(this->get_logger(), "History:");
        switch(qos_profile.history)
        {
            case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
                RCLCPP_INFO(this->get_logger(), "keep last");
                break;
            case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
                RCLCPP_INFO(this->get_logger(), "keep all");
                break;
            default:
                break;
        }
        RCLCPP_INFO(this->get_logger(), "Depth: %d", qos_profile.depth);
        RCLCPP_INFO(this->get_logger(), "Reliability:");
        switch(qos_profile.reliability)
        {
            case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
                RCLCPP_INFO(this->get_logger(), "reliable");
                break;
            case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
                RCLCPP_INFO(this->get_logger(), "best effort");
                break;
            default:
                break;
        }
        RCLCPP_INFO(this->get_logger(), "Durability:");
        switch(qos_profile.durability)
        {
            case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
                RCLCPP_INFO(this->get_logger(), "transient local");
                break;
            case RMW_QOS_POLICY_DURABILITY_VOLATILE:
                RCLCPP_INFO(this->get_logger(), "volatile");
                break;
            default:
                break;
        }
        RCLCPP_INFO(this->get_logger(), "Deadline time: sec=%ld, nsec=%ld", rmw_time_deadline.sec, rmw_time_deadline.nsec);
        RCLCPP_INFO(this->get_logger(), "Lifespan time: sec=%ld, nsec=%ld", rmw_time_lifespan.sec, rmw_time_lifespan.nsec);
    
        switch(qos_profile.liveliness)
        {
            case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
                RCLCPP_INFO(this->get_logger(), "automatic");
                break;
            case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
                RCLCPP_INFO(this->get_logger(), "manual by topic");
                break;
            default:
                break;
        }
        RCLCPP_INFO(this->get_logger(), "Lease duration: sec=%ld, nsec=%ld", rmw_time_liveliness.sec, rmw_time_liveliness.nsec);
    

        //
        // RCLCPP_INFO(this->get_logger(), "history=%d, depth=%d, reliablility=%d, durability=%d",
        //                         qos_profile.history,
        //                         qos_profile.depth,
        //                         qos_profile.reliability,
        //                         qos_profile.durability);
        // RCLCPP_INFO(this->get_logger(), "deadline time: sec=%ld, nsec=%ld", rmw_time_deadline.sec, rmw_time_deadline.nsec);
        // RCLCPP_INFO(this->get_logger(), "lifespan time: sec=%ld, nsec=%ld", rmw_time_lifespan.sec, rmw_time_lifespan.nsec);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Demo> node = std::make_shared<Demo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



double
rmw_time_to_seconds(const rmw_time_t & time)
{
  double result = static_cast<double>(time.sec);
  result += 1e-9 * time.nsec;
  return result;
}

void
print_qos(const rclcpp::QoS & qos)
{
  const auto & rmw_qos = qos.get_rmw_qos_profile();
  std::cout << "HISTORY POLICY: ";
  switch (rmw_qos.history) {
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      std::cout << "keep last";
      break;
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      std::cout << "keep all";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << " (depth: " << rmw_qos.depth << ')' << std::endl;

  std::cout << "RELIABILITY POLICY: ";
  switch (rmw_qos.reliability) {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      std::cout << "reliable";
      break;
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      std::cout << "best effort";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << std::endl;

  std::cout << "DURABILITY POLICY: ";
  switch (rmw_qos.durability) {
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      std::cout << "transient local";
      break;
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      std::cout << "volatile";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << std::endl;

  std::cout << "DEADLINE: " << rmw_time_to_seconds(rmw_qos.deadline) << std::endl;

  std::cout << "LIFESPAN: " << rmw_time_to_seconds(rmw_qos.lifespan) << std::endl;

  std::cout << "LIVELINESS POLICY: ";
  switch (rmw_qos.liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
      std::cout << "automatic";
      break;
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
      std::cout << "manual by topic";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << " (lease duration: " << rmw_time_to_seconds(rmw_qos.liveliness_lease_duration) <<
    ')' << std::endl;
}
