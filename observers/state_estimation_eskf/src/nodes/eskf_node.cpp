#include "../../include/state_estimation_eskf/eskf_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_estimator_eskf");
  ErrorStateKalmanFilterRos node;
  ros::spin();
}
