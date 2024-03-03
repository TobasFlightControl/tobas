#pragma once

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Quaternion& q);
