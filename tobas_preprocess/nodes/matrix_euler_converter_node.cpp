#include "../include/tobas_preprocess/matrix_euler_converter.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "matrix_euler_converter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_preprocess::MatrixEulerConverter node(nh, pnh);
  ros::spin();
}
