#include "../include/tobas_multirotor_controller/rotation_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_multirotor_rotation_controller");
  tobas_multirotor_controller::RotationControllerRos node;
  ros::spin();
}
