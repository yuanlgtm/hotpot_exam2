#include "rmsimulator_listener/rmsimulator_listener.hpp"

RmsimulatorListener::RmsimulatorListener() : Node("rmsimulator_listener"), queue_max_size_(10)
{
  // 初始化订阅者
  subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/target_point", 10, std::bind(&RmsimulatorListener::callback, this, std::placeholders::_1));

  // 设置定时器，每秒刷新队列
  timer_ = this->create_wall_timer(1s, std::bind(&RmsimulatorListener::updateQueue, this));
}

void RmsimulatorListener::callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) 
{
  latest_coordinate_ = *msg;
}

void RmsimulatorListener::updateQueue()
{
  // 如果队列已满，移除最老的数据
  if (coordinate_queue_.size() >= queue_max_size_) 
  {
    coordinate_queue_.pop();
  }
  
  // 将新接收到的数据添加进队列
  coordinate_queue_.push(latest_coordinate_);
  
  // 打印队列内容用于调试
  RCLCPP_INFO(this->get_logger(), "Queue size: %zu", coordinate_queue_.size());
  std::queue<geometry_msgs::msg::PointStamped> temp_queue = coordinate_queue_;
  while (!temp_queue.empty()) {
    geometry_msgs::msg::PointStamped point = temp_queue.front();
    temp_queue.pop();
    RCLCPP_INFO(this->get_logger(), "Point: x=%f, y=%f, z=%f, stamp=%ld.%ld", point.point.x, point.point.y, point.point.z, point.header.stamp.sec, point.header.stamp.nanosec);
  }
}

int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RmsimulatorListener>());
  rclcpp::shutdown();
  return 0;
}
