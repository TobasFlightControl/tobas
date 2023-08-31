#include "../include/tobas_multirotor_controller/rotation_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotation_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_multirotor_controller::RotationControllerRos node(nh, pnh);
  ros::spin();
}
