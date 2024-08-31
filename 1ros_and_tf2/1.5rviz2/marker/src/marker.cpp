#include "marker.hpp"

CoordinateTransformer::CoordinateTransformer(): Node("coordinate_transformer"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    // 订阅RMSimulator发布的敌人坐标
    subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/target_point", 10, std::bind(&CoordinateTransformer::callback, this, std::placeholders::_1));
    // 创建RViz中显示箭头的Marker发布者
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    queue_max_size_ = 10;
}

void CoordinateTransformer::callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) 
{
    // 第一步：左手坐标系到右手坐标系转换
    geometry_msgs::msg::PointStamped camera_frame_point;
    camera_frame_point.header = msg->header;
    camera_frame_point.header.frame_id = "camera_link";  // 确保设置正确的源坐标系名称
    camera_frame_point.point.x = msg->point.x;
    camera_frame_point.point.y = -msg->point.z;  // 交换并取负
    camera_frame_point.point.z = msg->point.y;   // 交换

    // 第二步：将坐标从相机光心坐标系转换到base_link坐标系
    try 
    {
        geometry_msgs::msg::PointStamped base_link_point;
        tf_buffer_.transform(camera_frame_point, base_link_point, "base_link", tf2::durationFromSec(1.0));

        // 将转换后的点添加到队列中
        if (coordinate_queue_.size() >= queue_max_size_) 
        {
            coordinate_queue_.pop();  // 如果队列满了，移除最老的点
        }
        coordinate_queue_.push(base_link_point);

        // 发布箭头marker
        if (coordinate_queue_.size() > 1) 
        {
            auto end_point = coordinate_queue_.back();
            auto start_point = coordinate_queue_.front();
            publishArrowMarker(start_point, end_point);
        }
    }
    catch (tf2::TransformException &ex) 
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
}

void CoordinateTransformer::publishArrowMarker(const geometry_msgs::msg::PointStamped& start_point, const geometry_msgs::msg::PointStamped& end_point) 
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "coordinate_transformer";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.points.push_back(start_point.point);
    marker.points.push_back(end_point.point);

    marker.scale.x = 0.1;  // 箭头直径
    marker.scale.y = 0.2;  // 箭头头部直径
    marker.scale.z = 0.0;  // 箭头长度（不需要设置）

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_publisher_->publish(marker);
}

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateTransformer>());
    rclcpp::shutdown();
    return 0;
}
