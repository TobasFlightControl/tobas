#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

#include <multirotor_msgs/LinearVelocity.h>

namespace gazebo
{
void timeGazeboToRos(const common::Time& g, ros::Time& r);
void timeRosToGazebo(const ros::Time& r, common::Time& g);

template <typename T>
void vectorGazeboToRos(const ignition::math::Vector3<T>& g, geometry_msgs::Vector3& r)
{
  r.x = g.X();
  r.y = g.Y();
  r.z = g.Z();
}

template <typename T>
void vectorRosToGazebo(const geometry_msgs::Vector3& r, ignition::math::Vector3<T>& g)
{
  g.X() = r.x;
  g.Y() = r.y;
  g.Z() = r.z;
}

template <typename T>
void pointGazeboToRos(const ignition::math::Vector3<T>& g, geometry_msgs::Point& r)
{
  r.x = g.X();
  r.y = g.Y();
  r.z = g.Z();
}

template <typename T>
void pointRosToGazebo(const geometry_msgs::Point& r, ignition::math::Vector3<T>& g)
{
  g.X() = r.x;
  g.Y() = r.y;
  g.Z() = r.z;
}

template <typename T>
void linvelGazeboToRos(const ignition::math::Vector3<T>& g, multirotor_msgs::LinearVelocity& r)
{
  r.vx = g.X();
  r.vy = g.Y();
  r.vz = g.Z();
}

template <typename T>
void linvelRosToGazebo(const multirotor_msgs::LinearVelocity& r, ignition::math::Vector3<T>& g)
{
  g.X() = r.vx;
  g.Y() = r.vy;
  g.Z() = r.vz;
}
}  // namespace gazebo
