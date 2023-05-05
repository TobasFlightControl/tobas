#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../../include/tobas_real/motors_handler_pwm.hpp"

#define INFO_PERIOD 1.

using namespace std;
using namespace dh_std;

MotorsHandler_PWM::MotorsHandler_PWM()
{
  if (getuid())
  {
    throw dh_ros::RuntimeError("Not root.");
  }

  getRosParams();

  for (const auto& rotor_config : rotor_configs_)
  {
    const uint32_t& pin = rotor_config.pin;
    uint32_t channel = getChannel(pin);

    if (!pwm_.initialize(channel))
    {
      throw dh_ros::RuntimeError("Failed to initialize RC output for PIN" + to_string(pin) + ".");
    }

    if (!pwm_.set_frequency(channel, rotor_config.pwm.frequency))
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

  registerSubscribers();
}

void MotorsHandler_PWM::getRosParams()
{
  dh_ros::getParam("/drone_name", drone_name_);
  dh_ros::getParam("/battery_voltage", battery_voltage_);
  dh_ros::getParam("/num_rotors", num_rotors_);
  getRotorConfigs(rotor_configs_);
}

void MotorsHandler_PWM::registerSubscribers()
{
  rotor_vels_sub_ = nh_.subscribe(
    "/" + drone_name_ + "/command/motor_speed", 1, &MotorsHandler_PWM::rotorSpeedsCb, this);
}

uint32_t MotorsHandler_PWM::getChannel(uint32_t pin)
{
  return pin - 1;
}

void MotorsHandler_PWM::rotorSpeedsCb(const tobas_msgs::RotorSpeeds& rotor_speeds)
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
    const auto& rotor_config = rotor_configs_[i];
    const double max_speed = rpmToRadPerSec(battery_voltage_ * rotor_config.kv);

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
    const auto& pin = rotor_config.pin;
    const auto& pwm = rotor_config.pwm;
    double period = remap(cmd_speed, 0., max_speed, pwm.min_pulse_width, pwm.max_pulse_width);
    if (!pwm_.set_duty_cycle(getChannel(pin), period))
    {
      throw dh_ros::RuntimeError("Failed to set PWM duty cycle for PIN" + to_string(pin) + ".");
    }
  }
}
