#include <ros/ros.h>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include "../../include/tobas_multirotor_controller/acceleration_controller.hpp"

#define WARN_PERIOD 1.

using namespace std;
using namespace KDL;

AccelerationController::AccelerationController(const Tree& tree)
  : gravity_(dh_ros::getParam<double>("/gravity")),
    battery_voltage_(dh_ros::getParam<double>("/battery_voltage")),
    rotor_configs_(getRotorConfigs()),
    inertia_solver_(tree)
{
  max_U_ = 0.;
  for (const auto& rotor_config : rotor_configs_)
  {
    const double max_speed = dh_std::rpmToRadPerSec(battery_voltage_ * rotor_config.kv);
    const double max_thrust = rotor_config.motor_constant * sqr(max_speed);
    max_U_ += max_thrust;
  }
}

void AccelerationController::updateInternalDataStructures()
{
  inertia_solver_.updateInternalDataStructures();
  mass_ = inertia_solver_.JntToMass();
}

void AccelerationController::update(
  const geometry_msgs::Vector3& acc_des,
  const double& yaw,
  double& U_out,
  double& roll_out,
  double& pitch_out)
{
  const double x = mass_ * acc_des.x;
  const double y = mass_ * acc_des.y;
  const double z = mass_ * (acc_des.z + gravity_);

  const double cos_yaw = cos(yaw);
  const double sin_yaw = sin(yaw);

  pitch_out = atan2(x * cos_yaw + y * sin_yaw, z);
  roll_out = atan2(cos(pitch_out) * (x * sin_yaw - y * cos_yaw), z);
  U_out = z / (cos(pitch_out) * cos(roll_out));

  if (U_out < 0. || max_U_ < U_out)
  {
    dh_ros::rosWarnThrottle(
      WARN_PERIOD,
      "U_out = " + to_string(U_out) + " is out of range [0, " + to_string(max_U_) + "].");
    U_out = dh_std::clamp(U_out, 0., max_U_);
  }
}
