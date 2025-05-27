#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_msgs_adapter/core.hpp>
#include <tobas_kdl_msgs_adapter/vector.hpp>
#include <tobas_msgs/msg/gnss.hpp>

namespace tobas_msgs
{
struct Gnss
{
  std_msgs::msg::Header header;
  uint8_t fix_type;
  double latitude;
  double longitude;
  double altitude;
  Eigen::Matrix3d position_covariance;
  kdl::Vector ground_speed;
  Eigen::Matrix3d velocity_covariance;

  using SharedPtr = std::shared_ptr<Gnss>;
  using ConstSharedPtr = std::shared_ptr<const Gnss>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::Gnss, tobas_msgs::msg::Gnss>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::Gnss;
  using ros_message_type = tobas_msgs::msg::Gnss;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.fix_type = src.fix_type;
    dst.latitude = src.latitude;
    dst.longitude = src.longitude;
    dst.altitude = src.altitude;
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.position_covariance, dst.position_covariance);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.ground_speed, dst.ground_speed);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.velocity_covariance, dst.velocity_covariance);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.fix_type = src.fix_type;
    dst.latitude = src.latitude;
    dst.longitude = src.longitude;
    dst.altitude = src.altitude;
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.position_covariance, dst.position_covariance);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.ground_speed, dst.ground_speed);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.velocity_covariance, dst.velocity_covariance);
  }
};

namespace tobas_msgs
{
using GnssAdapter = rclcpp::TypeAdapter<tobas_msgs::Gnss, tobas_msgs::msg::Gnss>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::Gnss, tobas_msgs::msg::Gnss);
