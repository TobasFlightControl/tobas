#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

#include <multirotor_msgs/LinearVelocity.h>

namespace gazebo
{
void timeGazeboToRos(const common::Time& g, ros::Time& r)
{
  r.sec = g.sec;
  r.nsec = g.nsec;
}

void timeRosToGazebo(const ros::Time& r, common::Time& g)
{
  g.sec = r.sec;
  g.nsec = r.nsec;
}

void vectorGazeboToRos(const ignition::math::Vector3d& g, geometry_msgs::Vector3& r)
{
  r.x = g.X();
  r.y = g.Y();
  r.z = g.Z();
}

void vectorRosToGazebo(const geometry_msgs::Vector3& r, ignition::math::Vector3d& g)
{
  g.X() = r.x;
  g.Y() = r.y;
  g.Z() = r.z;
}

void pointGazeboToRos(const ignition::math::Vector3d& g, geometry_msgs::Point& r)
{
  r.x = g.X();
  r.y = g.Y();
  r.z = g.Z();
}

void pointRosToGazebo(const geometry_msgs::Point& r, ignition::math::Vector3d& g)
{
  g.X() = r.x;
  g.Y() = r.y;
  g.Z() = r.z;
}

void linvelGazeboToRos(const ignition::math::Vector3d& g, multirotor_msgs::LinearVelocity& r)
{
  r.vx = g.X();
  r.vy = g.Y();
  r.vz = g.Z();
}

void linvelRosToGazebo(const multirotor_msgs::LinearVelocity& r, ignition::math::Vector3d& g)
{
  g.X() = r.vx;
  g.Y() = r.vy;
  g.Z() = r.vz;
}
}  // namespace gazebo
