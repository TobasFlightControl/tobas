#include <dh_ros_tools/rosparam.hpp>

#include "../../include/multirotor_tools/rotor_property.hpp"

using namespace std;

RotorProperties getRotorProperties()
{
  const string drone_name = dh_ros::getParam<string>("/drone_name");
  const int num_rotors = dh_ros::getParam<int>("/num_rotors");

  RotorProperties res(num_rotors);
  for (int i = 0; i < num_rotors; ++i)
  {
    const string rotor_prefix = "/rotor_" + to_string(i);

    // Link name
    res[i].link_name = dh_ros::getParam<string>(rotor_prefix + "/link_name");

    // Max velocity
    res[i].max_velocity = dh_ros::getParam<double>(rotor_prefix + "/max_velocity");
    if (res[i].max_velocity <= 0.)
    {
      throw dh_ros::RuntimeError(
        "Invalid max velocity: " + to_string(res[i].max_velocity) + " rad/s");
    }

    // Motor constant
    res[i].motor_constant = dh_ros::getParam<double>(rotor_prefix + "/motor_constant");
    if (res[i].motor_constant <= 0.)
    {
      throw dh_ros::RuntimeError(
        "Invalid motor constant: " + to_string(res[i].motor_constant) + " N*s^2/rad^2");
    }

    // Moment constant
    res[i].moment_constant = dh_ros::getParam<double>(rotor_prefix + "/moment_constant");
    if (res[i].moment_constant <= 0.)
    {
      throw dh_ros::RuntimeError(
        "Invalid moment constant: " + to_string(res[i].moment_constant) + " m");
    }

    // Direction
    const string direction = dh_ros::getParam<string>(rotor_prefix + "/direction");
    if (direction == "ccw")
    {
      res[i].direction = 1;
    }
    else if (direction == "cw")
    {
      res[i].direction = -1;
    }
    else
    {
      throw dh_ros::RuntimeError(
        "Invalid direction: " + direction + ". direction must be 'cw' or 'ccw'.");
    }

    // PIN
    res[i].pin = dh_ros::getParam<int>(rotor_prefix + "/pin");

    // PWM period range
    const double lb = dh_ros::getParam<int>(rotor_prefix + "/pwm_period/min");
    const double ub = dh_ros::getParam<int>(rotor_prefix + "/pwm_period/max");
    if (!(0. < lb < ub))
    {
      throw dh_ros::RuntimeError(
        "Invalid PWM range: (" + to_string(lb) + ", " + to_string(ub) + ") us");
    }
    res[i].pwm_range.lower = lb;
    res[i].pwm_range.upper = ub;
  }

  return res;
}
