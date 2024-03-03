#include "../include/orientation_estimation_complement/orientation_estimator_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orientation_estimator_complement");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  orientation_estimation_complement::OrientationEstimatorRos node(nh, pnh);
  ros::spin();
}
