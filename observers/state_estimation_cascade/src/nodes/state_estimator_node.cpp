#include "../../include/state_estimation_cascade/state_estimator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_estimator_cascade");
  state_estimation_cascade::StateEstimator node;
  ros::spin();
}
