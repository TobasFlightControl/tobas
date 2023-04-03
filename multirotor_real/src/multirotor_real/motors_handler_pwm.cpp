#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../../include/multirotor_real/motors_handler_pwm.hpp"

#define INFO_PERIOD 1.
#define FREQ 50.
#define PWM_LB 1000.
#define PWM_UB 2000.

using namespace std;
using namespace dh_std;

MotorsHandler_PWM::MotorsHandler_PWM(ros::NodeHandle& nh)
  : battery_voltage_(dh_ros::getParam<double>("/battery_voltage")),
    num_rotors_(dh_ros::getParam<int>("/num_rotors")),
    rotor_props_(getRotorProperties())
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

    if (!pwm_.set_frequency(channel, FREQ))
    {
      throw dh_ros::RuntimeError("Failed to set frequency for PIN" + to_string(pin) + ".");
    }

    if (!pwm_.enable(channel))
    {
      throw dh_ros::RuntimeError("RC output for PIN" + to_string(pin) + " is disabled.");
    }

    dh_ros::rosInfo("PWM output for PIN" + to_string(pin) + " is ready.");
    ros::Duration(0.2).sleep();  // 連続して設定を行うと失敗するため間隔をあける
  }

  string drone_name = dh_ros::getParam<string>("/drone_name");
  rotor_vels_sub_ = nh.subscribe(
    "/" + drone_name + "/command/motor_speed", 1, &MotorsHandler_PWM::rotorSpeedsCb, this);
}

uint32_t MotorsHandler_PWM::getChannel(uint32_t pin)
{
  return pin - 1;
}

void MotorsHandler_PWM::rotorSpeedsCb(const multirotor_msgs::RotorSpeeds& rotor_speeds)
{
  const auto& cmd_speeds = rotor_speeds.speeds;

  if (cmd_speeds.size() != num_rotors_)
  {
    dh_ros::rosErrorThrottle(
      INFO_PERIOD,
      "Size mismatch: " + to_string(cmd_speeds.size()) + " != " + to_string(num_rotors_));
    return;
  }

  for (int i = 0; i < num_rotors_; ++i)
  {
    const RotorProperty& prop = rotor_props_[i];
    const double max_speed = rpmToRadPerSec(battery_voltage_ * prop.kv * prop.efficiency);

    // 指令速度を決定
    double cmd_speed = cmd_speeds[i];
    if (cmd_speed < 0.)
    {
      dh_ros::rosErrorThrottle(
        INFO_PERIOD, "Rotor speed must be semi-positive: " + to_string(cmd_speed) + " < 0");
      cmd_speed = 0.;
    }
    else if (cmd_speed > max_speed)
    {
      dh_ros::rosErrorThrottle(
        INFO_PERIOD, "Commanded rotor speed is too large: " + to_string(cmd_speed) + " > "
                       + to_string(max_speed));
      cmd_speed = max_speed;
    }

    // パルス幅に変換して指令
    const uint32_t& pin = prop.pin;
    double period = remap(cmd_speed, 0., max_speed, PWM_LB, PWM_UB);
    if (!pwm_.set_duty_cycle(getChannel(pin), period))
    {
      throw dh_ros::RuntimeError("Failed to set PWM duty cycle for PIN" + to_string(pin) + ".");
    }
  }
}
