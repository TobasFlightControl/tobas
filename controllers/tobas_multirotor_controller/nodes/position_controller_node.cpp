#include "../include/tobas_multirotor_controller/position_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_controller");
  tobas_multirotor_controller::PositionControllerRos node;
  ros::spin();
}
