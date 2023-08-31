#include "../include/state_estimation_cascade/state_estimator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_estimator_cascade");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  state_estimation_cascade::StateEstimator node(nh, pnh);
  ros::spin();
}
