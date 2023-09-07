#include "../include/tobas_multirotor_controller/controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_multirotor_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_multirotor_controller::ControllerRos node(nh, pnh);
  ros::spin();
}
