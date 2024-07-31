#include "../include/tobas_mr_thrust_estimation/thrust_estimator.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "thrust_estimator");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_mr_thrust_estimation::ThrustEstimator node(node, pnh);
  rclcpp::spin();
}
