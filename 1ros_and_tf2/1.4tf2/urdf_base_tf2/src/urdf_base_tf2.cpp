#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <queue>

class CoordinateTransformer : public rclcpp::Node 
{
public:
    CoordinateTransformer(): Node("coordinate_transformer"),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_) 
    {
        // 订阅RMSimulator发布的敌人坐标
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/target_point", 10, std::bind(&CoordinateTransformer::callback, this, std::placeholders::_1));
        queue_max_size_ = 10;
    }
private:
    void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) 
    {
        // 第一步：左手坐标系到右手坐标系转换
        geometry_msgs::msg::PointStamped camera_frame_point;
        camera_frame_point.header = msg->header;
        camera_frame_point.header.frame_id = "camera_link";  // 确保设置正确的源坐标系名称
        camera_frame_point.point.x = msg->point.x;
        camera_frame_point.point.y = -msg->point.z;
        camera_frame_point.point.z = msg->point.y;  

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

            // 打印队列中的坐标用于调试
            RCLCPP_INFO(this->get_logger(), "Point in camera_link: x=%f, y=%f, z=%f",camera_frame_point.point.x, camera_frame_point.point.y, camera_frame_point.point.z);
            RCLCPP_INFO(this->get_logger(), "Point in base_link: x=%f, y=%f, z=%f",base_link_point.point.x, base_link_point.point.y, base_link_point.point.z);

        }
        catch (tf2::TransformException &ex) 
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::queue<geometry_msgs::msg::PointStamped> coordinate_queue_;
    size_t queue_max_size_;
};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateTransformer>());
    rclcpp::shutdown();
    return 0;
}
