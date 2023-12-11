#include "../include/tobas_task_space_control/effort_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_space_controller_effort");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_task_space_control::EffortControllerRos node(nh, pnh);
  ros::spin();
}
