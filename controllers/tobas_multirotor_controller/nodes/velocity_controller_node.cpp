#include "../include/tobas_multirotor_controller/velocity_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_controller");
  tobas_multirotor_controller::VelocityControllerRos node;
  ros::spin();
}
