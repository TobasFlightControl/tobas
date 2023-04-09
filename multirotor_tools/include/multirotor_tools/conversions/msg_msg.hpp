#pragma once

#include <geometry_msgs/Vector3.h>

#include <multirotor_msgs/LinearVelocity.h>
#include <multirotor_msgs/AngularVelocity.h>
#include <multirotor_msgs/LinearAccel.h>

namespace tf
{
void Vector3ToLinearVelocity(const geometry_msgs::Vector3& vec, multirotor_msgs::LinearVelocity& v);

void LinearVelocityToVector3(const multirotor_msgs::LinearVelocity& v, geometry_msgs::Vector3& vec);

void Vector3ToAngularVelocity(
  const geometry_msgs::Vector3& vec,
  multirotor_msgs::AngularVelocity& w);

void AngularVelocityToVector3(
  const multirotor_msgs::AngularVelocity& w,
  geometry_msgs::Vector3& vec);

void LinearAccelToVector3(const multirotor_msgs::LinearAccel& a, geometry_msgs::Vector3& vec);

void Vector3ToLinearAccel(const geometry_msgs::Vector3& vec, multirotor_msgs::LinearAccel& a);
}  // namespace tf
