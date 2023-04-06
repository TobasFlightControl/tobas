#include "../../include/imu_complementary_filter/complementary_filter_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_complementary_filter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ComplementaryFilterROS node(nh, nh_private);
  ros::spin();
}
