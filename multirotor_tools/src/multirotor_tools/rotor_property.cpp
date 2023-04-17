#include <dh_ros_tools/rosparam.hpp>

#include "../../include/multirotor_tools/rotor_property.hpp"

using namespace std;

RotorConfigs getRotorConfigs()
{
  const int num_rotors = dh_ros::getParam<int>("/num_rotors");
  RotorConfigs res(num_rotors);

  for (int i = 0; i < num_rotors; ++i)
  {
    const string rotor_prefix = "/rotor_" + to_string(i);

    // Link name
    res[i].link_name = dh_ros::getParam<string>(rotor_prefix + "/link_name");

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

    // KV
    res[i].kv = dh_ros::getParam<double>(rotor_prefix + "/kv");
    if (res[i].kv <= 0.)
    {
      throw dh_ros::RuntimeError("Invalid Kv: " + to_string(res[i].kv) + " rpm/V");
    }

    // Pin
    res[i].pin = dh_ros::getParam<int>(rotor_prefix + "/pin");
    if (res[i].pin < 1 || 14 < res[i].pin)
    {
      throw dh_ros::RuntimeError("Invalid rotor pin number: " + to_string(res[i].pin));
    }

    // ESC
    string esc_type = dh_ros::getParam<string>(rotor_prefix + "/esc_type");
    if (esc_type == "pwm")
    {
      res[i].esc_type = ESCType::PWM;
      res[i].pwm.frequency = dh_ros::getParam<double>(rotor_prefix + "/pwm/frequency");
      res[i].pwm.min_pulse_width = dh_ros::getParam<double>(rotor_prefix + "/pwm/min_pulse_width");
      res[i].pwm.max_pulse_width = dh_ros::getParam<double>(rotor_prefix + "/pwm/max_pulse_width");
    }
    else if (esc_type == "dshot")
    {
      res[i].esc_type = ESCType::DSHOT;
      // TODO
    }
    else
    {
      throw dh_ros::RuntimeError("Unknown ESC type: " + esc_type);
    }
  }

  return res;
}
