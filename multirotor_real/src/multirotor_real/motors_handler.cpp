#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../../include/multirotor_real/motors_handler.hpp"

#define INFO_PERIOD 1.

using namespace std;

MotorsHandler::MotorsHandler(ros::NodeHandle& nh)
  : num_rotors_(dh_ros::getParam<int>("/num_rotors", 0)), rotor_props_(getRotorProperties())
{
  if (getuid())
  {
    throw dh_ros::RuntimeError("Not root.");
  }

  for (const auto& rotor_prop : rotor_props_)
  {
    if (!pwm_.initialize(rotor_prop.pin))
    {
      throw dh_ros::RuntimeError(
        "Failed to initialize RC output for PIN" + to_string(rotor_prop.pin) + ".");
    }

    if (!pwm_.enable(rotor_prop.pin))
    {
      throw dh_ros::RuntimeError("RC output for PIN" + to_string(rotor_prop.pin) + " is disabled.");
    }
  }

  string drone_name = dh_ros::getParam<string>("/drone_name", "unknown");
  rotor_vels_sub_ =
    nh.subscribe("/" + drone_name + "/command/motor_speed", 1, &MotorsHandler::rotorSpeedsCb, this);
}

void MotorsHandler::rotorSpeedsCb(const multirotor_msgs::RotorSpeeds& rotor_speeds)
{
  const auto& speeds = rotor_speeds.speeds;

  if (speeds.size() != num_rotors_)
  {
    dh_ros::rosErrorThrottle(
      INFO_PERIOD, "Size mismatch: " + to_string(speeds.size()) + " != " + to_string(num_rotors_));
    return;
  }

  for (int i = 0; i < num_rotors_; ++i)
  {
    double angvel = speeds[i];
    const RotorProperty& prop = rotor_props_[i];

    if (angvel < 0. || prop.max_velocity < angvel)
    {
      dh_ros::rosErrorThrottle(
        INFO_PERIOD, "Invalid rotor velocity: " + to_string(angvel) + " rad/s");
      angvel = dh_std::clamp(angvel, 0., prop.max_velocity);
    }

    double period =
      dh_std::remap(angvel, 0., prop.max_velocity, prop.pwm_range.lower, prop.pwm_range.upper);

    pwm_.set_duty_cycle(prop.pin, period);
  }
}
