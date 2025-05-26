#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"

class TfBroadcasterNode : public rclcpp::Node
{
public:
    TfBroadcasterNode() : Node("tf_broadcaster_node")
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TfBroadcasterNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = rclcpp::Clock().now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "enu";

        transformStamped.transform.translation.x = 1.0;
        transformStamped.transform.translation.y = 2.0;
        transformStamped.transform.translation.z = 3.0;

        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfBroadcasterNode>());
    rclcpp::shutdown();
    return 0;
}


