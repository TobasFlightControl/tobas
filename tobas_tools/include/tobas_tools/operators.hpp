#pragma once

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

geometry_msgs::Vector3 operator*(const double& lhs, const geometry_msgs::Vector3& rhs);
geometry_msgs::Vector3 operator*(const geometry_msgs::Vector3& lhs, const double& rhs);

geometry_msgs::Vector3
operator+(const geometry_msgs::Vector3& lhs, const geometry_msgs::Vector3& rhs);
geometry_msgs::Vector3
operator-(const geometry_msgs::Vector3& lhs, const geometry_msgs::Vector3& rhs);

geometry_msgs::Vector3 operator-(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs);
