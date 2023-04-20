#pragma once

#include <geometry_msgs/Vector3.h>

#include <tobas_msgs/LinearVelocity.h>
#include <tobas_msgs/AngularVelocity.h>
#include <tobas_msgs/LinearAccel.h>

namespace tf
{
void Vector3ToLinearVelocity(const geometry_msgs::Vector3& vec, tobas_msgs::LinearVelocity& v);

void LinearVelocityToVector3(const tobas_msgs::LinearVelocity& v, geometry_msgs::Vector3& vec);

void Vector3ToAngularVelocity(
  const geometry_msgs::Vector3& vec,
  tobas_msgs::AngularVelocity& w);

void AngularVelocityToVector3(
  const tobas_msgs::AngularVelocity& w,
  geometry_msgs::Vector3& vec);

void LinearAccelToVector3(const tobas_msgs::LinearAccel& a, geometry_msgs::Vector3& vec);

void Vector3ToLinearAccel(const geometry_msgs::Vector3& vec, tobas_msgs::LinearAccel& a);
}  // namespace tf
