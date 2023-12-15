#include "../include/dynamixel_handler/dynamixel_handler.hpp"

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  dynamixel_handler::DynamixelHandler node(nh, pnh);
  ros::spin();
}
