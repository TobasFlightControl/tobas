#include "../../include/imu_complementary_filter/complementary_filter_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_complementary_filter");
  ComplementaryFilterRos node;
  ros::spin();
}
