#include "../include/tobas_mr_wind_estimation/wind_estimator.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "wind_estimator");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_mr_wind_estimation::WindEstimator node(node, pnh);
  rclcpp::spin();
}
