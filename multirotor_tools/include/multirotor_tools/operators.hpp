#pragma once

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

#include <multirotor_msgs/LinearVelocity.h>

geometry_msgs::Vector3 operator*(const double& lhs, const geometry_msgs::Vector3& rhs);

geometry_msgs::Vector3
operator+(const geometry_msgs::Vector3& lhs, const geometry_msgs::Vector3& rhs);

geometry_msgs::Vector3
operator-(const geometry_msgs::Vector3& lhs, const geometry_msgs::Vector3& rhs);

geometry_msgs::Vector3 operator-(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs);

geometry_msgs::Vector3
operator-(const multirotor_msgs::LinearVelocity& lhs, const multirotor_msgs::LinearVelocity& rhs);
