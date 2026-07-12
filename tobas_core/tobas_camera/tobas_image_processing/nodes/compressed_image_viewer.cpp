// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>

namespace tobas
{
namespace camera
{
class CompressedImageViewer : public BaseNode
{
public:
  explicit CompressedImageViewer(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());
  ~CompressedImageViewer();

private:
  void msgCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& _msg);

  ros2::SubscriberPtr<sensor_msgs::msg::CompressedImage> sub_;
};

CompressedImageViewer::CompressedImageViewer(const rclcpp::NodeOptions& _options)
  : BaseNode("compressed_image_viewer", _options)
{
  const auto compressed_image_topic = getStringParam("compressed_image_topic", "image_compressed");
  sub_ = createSubscriber(compressed_image_topic, &CompressedImageViewer::msgCb, this);
  cv::namedWindow("view", cv::WINDOW_NORMAL);
  cv::startWindowThread();
}

CompressedImageViewer::~CompressedImageViewer()
{
  cv::destroyWindow("view");
}

void CompressedImageViewer::msgCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg)
{
  try {
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);  // Convert the compressed image data to cv::Mat.
    cv::imshow("view", image);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e) {
    TOBAS_WARN("Could not convert to image!");
  }
}
}  // namespace camera
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::camera::CompressedImageViewer)
