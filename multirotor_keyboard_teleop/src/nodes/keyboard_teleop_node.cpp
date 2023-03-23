#include "../../include/multirotor_keyboard_teleop/keyboard_teleop.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle nh;
  CommandHandler node(nh);
  node.run();
}
