#pragma once

#include <iostream>
#include <geometry_msgs/msg/quaternion.hpp>

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Quaternion& q);
