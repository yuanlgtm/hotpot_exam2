#ifndef COORDINATE_TRANSFORMER_HPP
#define COORDINATE_TRANSFORMER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <queue>
#include <visualization_msgs/msg/marker.hpp>

class CoordinateTransformer : public rclcpp::Node 
{
public:
    CoordinateTransformer();

private:
    void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void publishArrowMarker(const geometry_msgs::msg::PointStamped& start_point, const geometry_msgs::msg::PointStamped& end_point);

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::queue<geometry_msgs::msg::PointStamped> coordinate_queue_;
    size_t queue_max_size_;
};

#endif // COORDINATE_TRANSFORMER_HPP
