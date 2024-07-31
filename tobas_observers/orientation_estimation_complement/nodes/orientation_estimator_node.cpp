#include "../include/orientation_estimation_complement/orientation_estimator_ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "orientation_estimator_complement");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  orientation_estimation_complement::OrientationEstimatorRos node(node, pnh);
  rclcpp::spin();
}
