#include "../include/tobas_multirotor_controller/position_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_multirotor_controller::PositionControllerRos node(nh, pnh);
  ros::spin();
}
