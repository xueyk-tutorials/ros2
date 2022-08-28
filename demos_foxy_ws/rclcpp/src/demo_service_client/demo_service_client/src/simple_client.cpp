
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std;
using namespace std::chrono_literals;


class SimpleClient : public rclcpp::Node
{
public:
    SimpleClient(std::string name="SimpleClient") : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "SimpleClient");

        // callback_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
        callback_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
        callback_group2 = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

        cli_add_two_ints = this->create_client<example_interfaces::srv::AddTwoInts>("simple/add_two_ints",
                                                                                    rmw_qos_profile_services_default,
                                                                                    callback_group);
        time_loop = this->create_wall_timer(1000ms, std::bind(&SimpleClient::loop, this), callback_group2);
    }

    void call_service()
    {
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = 1;
        request->b = 0;

        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future = cli_add_two_ints->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS){
            RCLCPP_ERROR(this->get_logger(), "Service call failed :(");
            assert(0);
        }

        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response = result_future.get();
        // RCLCPP_INFO(this->get_logger(), "sum=%d", response.sum);
        //
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        auto request_duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
        RCLCPP_INFO(this->get_logger(), "Got response in %ld microseconds", request_duration);
    }
    void loop()
    {
        static int cnt = 0;

        RCLCPP_INFO(this->get_logger(), "cnt=%d", cnt++);

        call_service();
    }

    void loop_while()
    {
        rclcpp::WallRate rate(1000ms);
        static int cnt = 0;

        while(rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(), "cnt2=%d", cnt++);

            call_service();
            //
            rate.sleep();
        }
    }


private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr cli_add_two_ints;

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group2;
    rclcpp::TimerBase::SharedPtr time_loop;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<SimpleClient> node = std::make_shared<SimpleClient>();

    /**
     * 不可以
     */
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);

    /**
     * 可以
     */
    // node->loop_while();

    rclcpp::shutdown();
    return 0;
}