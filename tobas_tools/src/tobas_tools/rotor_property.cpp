#include <dh_ros_tools/rosparam.hpp>

#include "../../include/tobas_tools/rotor_property.hpp"

using namespace std;

void getRotorConfigs(RotorConfigs& des)
{
  const int num_rotors = dh_ros::getParam<int>("/num_rotors");
  des.resize(num_rotors);

  for (int i = 0; i < num_rotors; ++i)
  {
    const string prefix = "/rotor_" + to_string(i);

    // Link name
    des[i].link_name = dh_ros::getParam<string>(prefix + "/link_name");

    // Motor constant
    des[i].motor_constant = dh_ros::getParam<double>(prefix + "/motor_constant");
    if (des[i].motor_constant <= 0.)
    {
      throw dh_ros::RuntimeError(
        "Invalid motor constant: " + to_string(des[i].motor_constant) + " N*s^2/rad^2");
    }

    // Moment constant
    des[i].moment_constant = dh_ros::getParam<double>(prefix + "/moment_constant");
    if (des[i].moment_constant <= 0.)
    {
      throw dh_ros::RuntimeError(
        "Invalid moment constant: " + to_string(des[i].moment_constant) + " m");
    }

    // Direction
    const string direction = dh_ros::getParam<string>(prefix + "/direction");
    if (direction == "ccw")
    {
      des[i].direction = 1;
    }
    else if (direction == "cw")
    {
      des[i].direction = -1;
    }
    else
    {
      throw dh_ros::RuntimeError(
        "Invalid direction: " + direction + ". direction must be 'cw' or 'ccw'.");
    }

    // KV
    des[i].kv = dh_ros::getParam<double>(prefix + "/kv");
    if (des[i].kv <= 0.)
    {
      throw dh_ros::RuntimeError("Invalid Kv: " + to_string(des[i].kv) + " rpm/V");
    }

    // Pin
    des[i].pin = dh_ros::getParam<int>(prefix + "/pin");
    if (des[i].pin < 1 || 14 < des[i].pin)
    {
      throw dh_ros::RuntimeError("Invalid rotor pin number: " + to_string(des[i].pin));
    }

    // ESC
    string esc_type = dh_ros::getParam<string>(prefix + "/esc_type");
    if (esc_type == "pwm")
    {
      des[i].esc_type = ESCType::PWM;
      des[i].pwm.frequency = dh_ros::getParam<double>(prefix + "/pwm/frequency");
      des[i].pwm.min_pulse_width = dh_ros::getParam<double>(prefix + "/pwm/min_pulse_width");
      des[i].pwm.max_pulse_width = dh_ros::getParam<double>(prefix + "/pwm/max_pulse_width");
    }
    else if (esc_type == "dshot")
    {
      des[i].esc_type = ESCType::DSHOT;
      // TODO
    }
    else
    {
      throw dh_ros::RuntimeError("Unknown ESC type: " + esc_type);
    }
  }
}
