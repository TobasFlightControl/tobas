#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../../include/multirotor_real/motors_handler_dshot.hpp"

#define PWM_FREQ 50.
#define INFO_PERIOD 1.

using namespace std;

MotorsHandler_DSHOT::MotorsHandler_DSHOT(ros::NodeHandle& nh)
  : num_rotors_(dh_ros::getParam<int>("/num_rotors")), rotor_props_(getRotorProperties())
{
  if (getuid())
  {
    throw dh_ros::RuntimeError("Not root.");
  }

  for (const auto& rotor_prop : rotor_props_)
  {
    const uint32_t& pin = rotor_prop.pin;
    uint32_t channel = getChannel(pin);

    if (!pwm_.initialize(channel))
    {
      throw dh_ros::RuntimeError("Failed to initialize RC output for PIN" + to_string(pin) + ".");
    }

    if (!pwm_.enable(channel))
    {
      throw dh_ros::RuntimeError("RC output for PIN" + to_string(pin) + " is disabled.");
    }

    dh_ros::rosInfo("PWM output for PIN" + to_string(pin) + " is ready.");
    ros::Duration(0.2).sleep();  // 連続して設定を行うと失敗するため間隔をあける
  }

  string drone_name = dh_ros::getParam<string>("/drone_name");
  rotor_vels_sub_ =
    nh.subscribe("/" + drone_name + "/command/motor_speed", 1, &MotorsHandler_DSHOT::rotorSpeedsCb, this);
}

uint32_t MotorsHandler_DSHOT::getChannel(uint32_t pin)
{
  return pin - 1;
}

void MotorsHandler_DSHOT::rotorSpeedsCb(const multirotor_msgs::RotorSpeeds& rotor_speeds)
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

    if (angvel < 0. || prop.max_velocity + 1. < angvel)
    {
      dh_ros::rosErrorThrottle(
        INFO_PERIOD, "Invalid rotor velocity: " + to_string(angvel) + " rad/s is out of ["
                       + to_string(0.) + ", " + to_string(prop.max_velocity) + "] rad/s.");
      angvel = dh_std::clamp(angvel, 0., prop.max_velocity);
    }

    const uint32_t& pin = prop.pin;
    double period =
      dh_std::remap(angvel, 0., prop.max_velocity, prop.pwm_range.lower, prop.pwm_range.upper);

    if (!pwm_.set_duty_cycle(getChannel(pin), period))
    {
      throw dh_ros::RuntimeError("Failed to set PWM duty cycle for PIN" + to_string(pin) + ".");
    }
  }
}
