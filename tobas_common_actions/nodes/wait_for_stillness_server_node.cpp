#include "../include/tobas_common_actions/wait_for_stillness_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wait_for_stillness_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_common_actions::WaitForStillnessServer node(nh, pnh);
  ros::spin();
}
