#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/rate.hpp>

#include "../../include/tobas_real/motors_handler_dshot.hpp"

#define INFO_PERIOD 1.

using namespace std;
using namespace dh_std;

MotorsHandler_DSHOT::MotorsHandler_DSHOT() : cmd_received_(false), dshot_(DSHOT::DSHOT_600)
{
  if (getuid())
  {
    throw dh_ros::RuntimeError("Not root.");
  }

  getRosParams();

  for (const auto& rotor_config : rotor_configs_)
  {
    dshot_.initialize(rotor_config.pin);
  }

  registerSubscribers();
}

void MotorsHandler_DSHOT::run()
{
  dh_ros::Rate rate(update_rate_);

  while (ros::ok())
  {
    if (!cmd_received_)
    {
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    for (int i = 0; i < num_rotors_; ++i)
    {
      const RotorConfig& rotor_config = rotor_configs_[i];
      const double max_speed = rpmToRadPerSec(battery_voltage_ * rotor_config.kv);

      // 指令速度を決定
      double cmd_speed = cmd_speeds_[i];
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

      // スロットルに変換して指令
      uint32_t throttle = remap<double>(cmd_speed, 0., max_speed, 48, (1 << 11) - 1);
      dshot_.setSignal(rotor_config.pin, throttle);
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void MotorsHandler_DSHOT::getRosParams()
{
  dh_ros::getParam("/drone_name", drone_name_);
  dh_ros::getParam("/battery_voltage", battery_voltage_);
  dh_ros::getParam("/num_rotors", num_rotors_);
  getRotorConfigs(rotor_configs_);

  dh_ros::getParam("~update_rate", update_rate_, kDefaultUpdateRate);
}

void MotorsHandler_DSHOT::registerSubscribers()
{
  rotor_vels_sub_ = nh_.subscribe(
    "/" + drone_name_ + "/command/motor_speed", 1, &MotorsHandler_DSHOT::rotorSpeedsCb, this);
}

void MotorsHandler_DSHOT::rotorSpeedsCb(const tobas_msgs::RotorSpeeds& rotor_speeds)
{
  const auto& speeds = rotor_speeds.speeds;

  if (speeds.size() != num_rotors_)
  {
    dh_ros::rosErrorThrottle(
      INFO_PERIOD, "Size mismatch: " + to_string(speeds.size()) + " != " + to_string(num_rotors_));
    return;
  }

  if (!cmd_received_)
  {
    cmd_received_ = true;
  }

  cmd_speeds_ = speeds;
}
