#include <dh_ros_tools/rosparam.hpp>

#include "../../include/tobas_tools/rotor_property.hpp"

using namespace std;

void getRotorConfigs(RotorConfigs& des)
{
  int num_rotors;
  dh_ros::getParam("/num_rotors", num_rotors);
  des.resize(num_rotors);

  for (int i = 0; i < num_rotors; ++i)
  {
    const string prefix = "/rotor_" + to_string(i);

    // Link name
    dh_ros::getParam(prefix + "/link_name", des[i].link_name);

    // Motor constant
    dh_ros::getParam(prefix + "/motor_constant", des[i].motor_constant);
    if (des[i].motor_constant <= 0.)
    {
      throw dh_ros::RuntimeError(
        "Invalid motor constant: " + to_string(des[i].motor_constant) + " N*s^2/rad^2");
    }

    // Moment constant
    dh_ros::getParam(prefix + "/moment_constant", des[i].moment_constant);
    if (des[i].moment_constant <= 0.)
    {
      throw dh_ros::RuntimeError(
        "Invalid moment constant: " + to_string(des[i].moment_constant) + " m");
    }

    // Direction
    string direction;
    dh_ros::getParam(prefix + "/direction", direction);
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
    dh_ros::getParam(prefix + "/kv", des[i].kv);
    if (des[i].kv <= 0.)
    {
      throw dh_ros::RuntimeError("Invalid Kv: " + to_string(des[i].kv) + " rpm/V");
    }

    // Pin
    dh_ros::getParam(prefix + "/pin", des[i].pin);
    if (des[i].pin < 1 || 14 < des[i].pin)
    {
      throw dh_ros::RuntimeError("Invalid rotor pin number: " + to_string(des[i].pin));
    }

    // ESC
    string esc_type;
    dh_ros::getParam(prefix + "/esc_type", esc_type);
    if (esc_type == "pwm")
    {
      des[i].esc_type = ESCType::PWM;
      dh_ros::getParam(prefix + "/pwm/frequency", des[i].pwm.frequency);
      dh_ros::getParam(prefix + "/pwm/min_pulse_width", des[i].pwm.min_pulse_width);
      dh_ros::getParam(prefix + "/pwm/max_pulse_width", des[i].pwm.max_pulse_width);
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
