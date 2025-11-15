#include "tobas_img_processing/image_decompressor.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp_components/register_node_macro.hpp>

ImageDecompressor::ImageDecompressor(const rclcpp::NodeOptions& _options)
  : tobas::BaseNode("image_decompressor", _options)
{
  std::string subscribed_compressed_image_topic =
    getStringParam("subscribed_compressed_image_topic", std::string("image_compressed"));
  std::string published_image_topic = getStringParam("published_image_topic", std::string("image_decompressed"));

  pub_ = createPublisher<sensor_msgs::msg::Image>(published_image_topic);
  sub_ = createSubscriber(subscribed_compressed_image_topic, &ImageDecompressor::msgCb, this);
}

void ImageDecompressor::msgCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->header = msg->header;
    cv_ptr->image = cv::imdecode(cv::Mat(msg->data), 1);
    switch (cv_ptr->image.channels()) {
      case 1:
        cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
        break;
      case 3:
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unsupported number of channels: %i", cv_ptr->image.channels());
        break;
    }
    pub_->publish(*cv_ptr->toImageMsg());
  }
  catch (cv_bridge::Exception& e) {
    RCLCPP_WARN(this->get_logger(), "Could not convert to image!");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImageDecompressor)
