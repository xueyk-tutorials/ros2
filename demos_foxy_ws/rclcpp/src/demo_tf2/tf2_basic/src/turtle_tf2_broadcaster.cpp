#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>

class FramePublisher : public rclcpp::Node
{
public:
    FramePublisher () : Node("turtle_tf2_frame_publisher")
    {
        this->declare_parameter<std::string>("turtlename", "turtle1");
        this->get_parameter("turtlename", turtlename_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::ostringstream stream;
        stream << "/" << turtlename_.c_str() << "/pose";
        std::string topic_name = stream.str();

        subscription_ = this->create_subscription<turtlesim::msg::Pose>(topic_name,
                                                                        10,
                                                                        std::bind(&FramePublisher::callback_turtle_pose, this, std::placeholders::_1));
    }
private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string turtlename_;
    void callback_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
    {
        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t;

        //
        t.header.stamp = now;
        t.header.frame_id = "world";
        t.child_frame_id = turtlename_.c_str();

        //
        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;

        //
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);        
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}