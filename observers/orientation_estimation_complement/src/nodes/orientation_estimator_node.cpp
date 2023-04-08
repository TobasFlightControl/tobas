#include "../../include/orientation_estimation_complement/orientation_estimator_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orientation_estimation_complement");
  OrientationEstimatorRos node;
  ros::spin();
}
