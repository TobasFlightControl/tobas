#include "../include/state_estimation_eskf/eskf_ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "state_estimator_eskf");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  state_estimation_eskf::ErrorStateKalmanFilterRos node(node, pnh);
  rclcpp::spin();
}
