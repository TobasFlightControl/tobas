#include "../include/tobas_rotor_controller/rotor_controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotor_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_rotor_controller::RotorController node(nh, pnh);
  ros::spin();
}
