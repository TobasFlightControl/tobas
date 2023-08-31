#include "../include/tobas_multirotor_controller/velocity_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_multirotor_controller::VelocityControllerRos node(nh, pnh);
  ros::spin();
}
