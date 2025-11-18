#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>


class CompressedImageViewer : public tobas::BaseNode
{
public:
  explicit CompressedImageViewer(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());
  ~CompressedImageViewer();

private:
  void msgCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& _msg);

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
};


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
    cv::imshow("view", image);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e) {
    TOBAS_WARN("Could not convert to image!");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(CompressedImageViewer)
