#include "tobas_img_processing/compressed_image_viewer.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>

CompressedImageViewer::CompressedImageViewer(const rclcpp::NodeOptions& _options)
  : tobas::BaseNode("compressed_image_viewer", _options)
{
  const std::string compressed_image_topic = getStringParam("compressed_image_topic", std::string("image_compressed"));
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
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);  // convert compressed image data to cv::Mat
    // std::cout << "cols : " << image.cols << ", rows : " << image.rows <<", size : " << msg->data.size() << ",
    // elemSize : " << image.elemSize() << ", total : " << image.total() << std::endl;
    cv::imshow("view", image);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e) {
    RCLCPP_WARN(this->get_logger(), "Could not convert to image!");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(CompressedImageViewer)
