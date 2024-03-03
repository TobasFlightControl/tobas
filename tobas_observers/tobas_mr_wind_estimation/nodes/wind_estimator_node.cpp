#include "../include/tobas_mr_wind_estimation/wind_estimator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wind_estimator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mr_wind_estimation::WindEstimator node(nh, pnh);
  ros::spin();
}
