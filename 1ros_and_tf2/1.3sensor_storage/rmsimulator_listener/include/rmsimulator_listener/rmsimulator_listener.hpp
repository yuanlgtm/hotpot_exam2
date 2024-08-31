#ifndef RMSIMULATOR_LISTENER_HPP
#define RMSIMULATOR_LISTENER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <queue>
#include <chrono>

using namespace std::chrono_literals;

class RmsimulatorListener : public rclcpp::Node 
{
public:
  RmsimulatorListener();

private:
  void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void updateQueue();

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 用于存储敌人坐标的队列
  std::queue<geometry_msgs::msg::PointStamped> coordinate_queue_;
  // 用于存储最近接收到的坐标
  geometry_msgs::msg::PointStamped latest_coordinate_;
  // 队列的最大长度
  const size_t queue_max_size_;
};

#endif // RMSIMULATOR_LISTENER_HPP
