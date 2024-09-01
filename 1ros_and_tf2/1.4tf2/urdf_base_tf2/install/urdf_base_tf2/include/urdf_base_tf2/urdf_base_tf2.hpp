#ifndef URDF_BASE_TF2_HPP
#define URDF_BASE_TF2_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <queue>

class CoordinateTransformer : public rclcpp::Node 
{
public:
    CoordinateTransformer();

private:
    void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::queue<geometry_msgs::msg::PointStamped> coordinate_queue_;
    size_t queue_max_size_;
};

#endif // URDF_BASE_TF2_HPP
