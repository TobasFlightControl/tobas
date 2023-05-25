#include "../../include/tobas_keyboard_teleop/keyboard_teleop.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_teleop");
  tobas_keyboard_teleop::CommandHandler node;
  node.run();
}
