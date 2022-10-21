#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/header.hpp"


using namespace std::chrono_literals;

class Producer : public rclcpp::Node
{
public:
    Producer(const std::string &name, const std::string & input)
     : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        // 创建发布者，input为topic名
        this->pub_header_ = this->create_publisher<std_msgs::msg::Header>(input, 10);
        // 创建于pub_header_同类型的weak_ptr指针
        std::weak_ptr<std::remove_pointer<decltype(pub_header_.get())>::type> captured_pub_header = pub_header_;

        // 定时器回调函数，周期性执行，1Hz
        auto callback = [captured_pub_header]() -> void{
            auto pub_ptr = captured_pub_header.lock();
            if(!pub_ptr)
            {
                return;
            }
            static int32_t count = 0;
            // 创建发布的消息，必须使用UniquePtr
            std_msgs::msg::Header::UniquePtr msg(new std_msgs::msg::Header());
            msg->stamp = rclcpp::Clock().now();
            printf(
                ">>>Published message stamp:sec=%d, nanosec=%d, and address:0x%" PRIXPTR "\n",
                msg->stamp.sec, msg->stamp.nanosec,
                reinterpret_cast<std::uintptr_t>(msg.get()));

            // 发布时必须使用std::move
            pub_ptr->publish(std::move(msg));
        };
        timer_ = this->create_wall_timer(1s, callback);
    }
private:
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_header_;
    rclcpp::TimerBase::SharedPtr timer_;
};

class Consumer : public rclcpp::Node
{
public:
    Consumer(const std::string &name, const std::string & input)
      : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        sub_header_ =  this->create_subscription<std_msgs::msg::Header>(
            input,
            10,
            [](std_msgs::msg::Header::UniquePtr msg){
                printf(
                    "<<<Received message stamp:sec=%d, nanosec=%d, and address:0x%" PRIXPTR "\n",
                    msg->stamp.sec, msg->stamp.nanosec,
                    reinterpret_cast<std::uintptr_t>(msg.get()));
            }
        );
    }
private:
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_header_;
};


int main(int argc, char *argv[])
{   
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    // 需要通信的节点必须要放到同一个进程中！！！
    rclcpp::executors::SingleThreadedExecutor executor;
    auto producer = std::make_shared<Producer>("producer", "number");
    auto consumer = std::make_shared<Consumer>("consumer", "number");

    executor.add_node(producer);
    executor.add_node(consumer);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
