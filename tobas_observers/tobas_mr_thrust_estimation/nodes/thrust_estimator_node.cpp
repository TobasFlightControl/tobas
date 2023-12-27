#include "../include/tobas_mr_thrust_estimation/thrust_estimator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thrust_estimator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mr_thrust_estimation::ThrustEstimator node(nh, pnh);
  ros::spin();
}
