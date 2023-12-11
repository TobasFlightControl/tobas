#include "../include/tobas_task_space_control/velocity_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_controller_velocity");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_task_space_control::VelocityControllerRos node(nh, pnh);
  ros::spin();
}
