#pragma once

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>

geometry_msgs::msg::Vector3 operator*(const double& lhs, const geometry_msgs::msg::Vector3& rhs);
geometry_msgs::msg::Vector3 operator*(const geometry_msgs::msg::Vector3& lhs, const double& rhs);
geometry_msgs::msg::Vector3 operator/(const double& lhs, const geometry_msgs::msg::Vector3& rhs);
geometry_msgs::msg::Vector3 operator/(const geometry_msgs::msg::Vector3& lhs, const double& rhs);

geometry_msgs::msg::Vector3 operator+(const geometry_msgs::msg::Vector3& lhs, const geometry_msgs::msg::Vector3& rhs);
geometry_msgs::msg::Vector3 operator-(const geometry_msgs::msg::Vector3& lhs, const geometry_msgs::msg::Vector3& rhs);

geometry_msgs::msg::Vector3 operator-(const geometry_msgs::msg::Point& lhs, const geometry_msgs::msg::Point& rhs);
