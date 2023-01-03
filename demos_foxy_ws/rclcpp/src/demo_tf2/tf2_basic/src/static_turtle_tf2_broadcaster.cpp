#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster_node.hpp>

class StaticFramePublisher : public rclcpp::Node
{
public:
    explicit StaticFramePublisher(char *transformation[]) : Node("static_turtle_tf2_broadcaster")
    {
        tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // publish static transforms once at startup
        this->make_transforms(transformation);
    }
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
    void make_transforms(char *transformation[])
    {
        rclcpp::Time now;
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "world";
        t.child_frame_id = transformation[1];

        t.transform.translation.x = atof(transformation[2]);
        t.transform.translation.y = atof(transformation[3]);
        t.transform.translation.z = atof(transformation[4]);

        tf2::Quaternion q;
        q.setRPY(
            atof(transformation[5]),
            atof(transformation[6]),
            atof(transformation[7]));
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // RCLCPP_INFO(this->get_logger(), "roll=%.2f, pitch=%.2f, yaw=%.2f", atof(transformation[5], atof(transformation[6], atof(transformation[7]);
        RCLCPP_INFO(this->get_logger(), "q(x,y,z,w)=%.2f,%.2f,%.2f,%.2f", q.x(), q.y(),q.z(),q.w());

        tf_publisher_->sendTransform(t);
    }
};


int main(int argc, char *argv[])
{
    auto logger = rclcpp::get_logger("logger");

    if(argc != 8)
    {
        RCLCPP_INFO(
            logger, "Invalid number of parameters\n usage: "
            "ros2 run demo_tf2 static_turtle_tf2_broadcaster "
            "child_frame_name x y z roll pitch yaw"
        );
        return 1;
    }

    if(strcmp(argv[1], "world") == 0)
    {
        RCLCPP_INFO(logger, "Your static turtle name cannot be `world`");
        return 1;
    }
    rclcpp::init(argc,  argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
    rclcpp::shutdown();

    return 0;
}