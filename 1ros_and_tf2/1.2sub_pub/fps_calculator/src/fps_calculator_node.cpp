#include "fps_calculator.hpp"

FpsCalculatorNode::FpsCalculatorNode() : Node("fps_calculator_node"), image_count_(0), fps_(0.0)
{
  // 创建图像订阅者
  image_sub_ = image_transport::create_subscription(this, "/image_raw", std::bind(&FpsCalculatorNode::imageCallback, this, std::placeholders::_1), "raw");
  
  // 创建图像发布者
  image_pub_ = image_transport::create_publisher(this, "/image_with_fps");
  
  // 创建定时器，每秒触发一次
  timer_ = this->create_wall_timer(1s, std::bind(&FpsCalculatorNode::timerCallback, this));
}

void FpsCalculatorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  // 转换ROS图像消息到OpenCV图像
  cv::Mat img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
  
  // 增加图像计数
  image_count_++;
  
  // 在图像上显示帧率
  std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps_));
  cv::putText(img, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
  
  // 发布带有帧率的图像
  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  image_pub_.publish(msg);
}

void FpsCalculatorNode::timerCallback()
{
  // 计算帧率
  fps_ = static_cast<double>(image_count_);
  
  // 重置图像计数
  image_count_ = 0;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FpsCalculatorNode>());
  rclcpp::shutdown();
  return 0;
}
