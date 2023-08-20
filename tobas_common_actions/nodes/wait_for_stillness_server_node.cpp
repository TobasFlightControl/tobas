#include "../include/tobas_common_actions/wait_for_stillness_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wait_for_stillness_server");
  tobas_common_actions::WaitForStillnessServer node;
  ros::spin();
}
