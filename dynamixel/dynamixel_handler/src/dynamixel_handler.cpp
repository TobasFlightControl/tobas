#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../include/dynamixel_handler/dynamixel_handler.hpp"
#include "../include/dynamixel_handler/constants.hpp"

using namespace std;

namespace dynamixel_handler
{
DynamixelHandler::DynamixelHandler(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : nh_(nh), pnh_(pnh), name_(name)
{
  // Get ROS parameters
  getRosParams();

  // Initialize Dynamixel SDK
  poh_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
  pah_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  // Open serial port
  if (!poh_->openPort())
    ROS_THROW_NAMED(name_, "Failed to open port '" << device_name_ << "'");

  // Set baudrate
  if (!poh_->setBaudRate(baudrate_))
    ROS_THROW_NAMED(name_, "Failed to set baudrate to " << baudrate_);

  // Get motor configurations
  getMotorConfigs();

  for (const auto& [name, cfg] : motors_)
  {
    // Disable torque
    if (pah_->write1ByteTxRx(poh_, cfg.id, kAddrToruqeEnable, kTorqueDisable) < 0)
      ROS_THROW_NAMED(name_, "Failed to disable torque of '" << name << "'.");

    // Set operating mode
    if (pah_->write1ByteTxRx(poh_, cfg.id, kAddrOperatingMode, cfg.operating_mode) < 0)
      ROS_THROW_NAMED(name_, "Failed to set operating mode of '" << name << "'.");

    // Enable torque
    if (pah_->write1ByteTxRx(poh_, cfg.id, kAddrToruqeEnable, kTorqueEnable) < 0)
      ROS_THROW_NAMED(name_, "Failed to enable torque of '" << name << "'.");

    rosInfo(name_, "'" << name << "' is initialized.");
  }

  // Register publishers and subscribers
  registerPubSub();

  // Start main timer
  main_timer_ = nh_.createTimer(update_rate_, &self::mainTimerCb, this);
}

DynamixelHandler::~DynamixelHandler()
{
  // Disable torque
  for (const auto& [name, cfg] : motors_)
    pah_->write1ByteTxRx(poh_, cfg.id, kAddrToruqeEnable, kTorqueDisable);

  // Close serial port
  poh_->closePort();
}

void DynamixelHandler::getRosParams()
{
  dh_ros::getParam(pnh_, "joint_names", jnt_names_);
  dh_ros::getParam(pnh_, "device_name", device_name_, string(kDefaultDeviceName));
  dh_ros::getParam(pnh_, "protocol_version", protocol_version_, kDefaultProtocolVersion);
  dh_ros::getParam(pnh_, "baudrate", baudrate_, kDefaultBaudRate);
  dh_ros::getParam(pnh_, "update_rate", update_rate_, kDefaultUpdateRate);
}

void DynamixelHandler::getMotorConfigs()
{
  DynamixelConfig cfg;
  string operating_mode;
  unordered_set<uint8_t> used_ids;
  uint16_t model_number;

  for (const auto& name : jnt_names_)
  {
    // ID
    dh_ros::getParam(pnh_, name + "/id", cfg.id, kDefaultId);
    if (used_ids.contains(cfg.id))
    {
      rosError(name_, "Motor ID " << static_cast<int>(cfg.id) << " is duplicated.");
      continue;
    }
    used_ids.insert(cfg.id);

    // Current関連
    if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrModelNumber, &model_number) < 0)
    {
      rosError(name_, "Failed to get model number of '" << name << "'.");
      continue;
    }
    switch (model_number)
    {
      case kModelNumber_XL430W250:
        cfg.current_available = false;
        break;
      case kModelNumber_XC430W250:
        cfg.current_available = false;
        break;
      case kModelNumber_XM430W350:
        cfg.current_available = true;
        cfg.current_scaling_factor = 2.69;
        break;
      default:
        rosWarn(name_, "Setting for model number " << model_number << " is not implemented yet.");
        cfg.current_available = false;
        break;
    }

    // Operating mode
    dh_ros::getParam(pnh_, name + "/operating_mode", operating_mode, string(kDefaultOperatingMode));
    if (operating_mode == "current")
    {
      if (!cfg.current_available)
      {
        rosError(name_, "Current control mode is unavailable for '" << name << "'.");
        continue;
      }
      cfg.operating_mode = kPositionControlMode;
    }
    else if (operating_mode == "velocity")
      cfg.operating_mode = kVelocityControlMode;
    else if (operating_mode == "position")
      cfg.operating_mode = kPositionControlMode;
    else if (operating_mode == "extended_position")
      cfg.operating_mode = kExtendedPositionControlMode;
    else if (operating_mode == "current_base_position")
    {
      if (!cfg.current_available)
      {
        rosError(name_, "Current-base position control mode is unavailable for '" << name << "'.");
        continue;
      }
      cfg.operating_mode = kCurrentBasePositionControlMode;
    }
    else if (operating_mode == "pwm")
    {
      cfg.operating_mode = kPwmControlMode;
    }
    else
    {
      rosError(name_, "Unknown operating mode for '" << name << "'.");
      continue;
    }

    // Insert motor config
    motors_[name] = cfg;
  }
}

void DynamixelHandler::registerPubSub()
{
  motor_states_pub_ = nh_.advertise<dynamixel_msgs::MotorStates>("motor_states", 1);
  cur_js_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

  tar_js_sub_ = nh_.subscribe("command/joint_states", 1, &self::targetJointStateCb, this);
}

void DynamixelHandler::publishCurrentStates(const ros::Time& cur_time)
{
  const auto motor_states = boost::make_shared<dynamixel_msgs::MotorStates>();
  const auto cur_js = boost::make_shared<sensor_msgs::JointState>();

  motor_states->header.stamp = cur_time;
  cur_js->header.stamp = cur_time;

  for (const auto& [name, cfg] : motors_)
  {
    if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrPresentPwm, (uint16_t*)&present_pwm_) == 0)
    {
      motor_state_.pwm = static_cast<double>(present_pwm_) * 100 / 855;
    }
    else
    {
      rosError(name_, "Failed to get present PWM of '" << name << "'.");
      continue;
    }

    if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrPresentCurrent, (uint16_t*)&present_current_) == 0)
    {
      if (cfg.current_available)
      {
        motor_state_.current =
          static_cast<double>(present_current_) * cfg.current_scaling_factor / 1000;
        motor_state_.load = nan(kUnavailable);
      }
      else
      {
        motor_state_.current = nan(kUnavailable);
        motor_state_.load = static_cast<double>(present_current_) * 0.1;
      }
    }
    else
    {
      rosError(name_, "Failed to get present current of '" << name << "'.");
      continue;
    }

    if (pah_->read4ByteTxRx(poh_, cfg.id, kAddrPresentPosition, (uint32_t*)&present_position_) == 0)
    {
      motor_state_.position = dh_std::remap<double>(present_position_, 0, 1 << 12, -M_PI, M_PI);
    }
    else
    {
      rosError(name_, "Failed to get present position of '" << name << "'.");
      continue;
    }

    if (pah_->read4ByteTxRx(poh_, cfg.id, kAddrPresentVelocity, (uint32_t*)&present_velocity_) == 0)
    {
      motor_state_.velocity = static_cast<double>(present_velocity_) * kVelocityDecodeFactor;
    }
    else
    {
      rosError(name_, "Failed to get present velocity of '" << name << "'.");
      continue;
    }

    if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrPresentInputVoltage, &present_input_voltage_) == 0)
    {
      motor_state_.input_voltage = static_cast<double>(present_input_voltage_) * 0.1;
    }
    else
    {
      rosError(name_, "Failed to get present input voltage of '" << name << "'.");
      continue;
    }

    if (pah_->read1ByteTxRx(poh_, cfg.id, kAddrPresentTemperatur, &present_temperature_) == 0)
    {
      motor_state_.temperature = static_cast<double>(present_temperature_) * 1;
    }
    else
    {
      rosError(name_, "Failed to get present temperature of '" << name << "'.");
      continue;
    }

    motor_states->names.push_back(name);
    motor_states->states.push_back(motor_state_);

    cur_js->name.push_back(name);
    cur_js->position.push_back(motor_state_.position);
    cur_js->velocity.push_back(motor_state_.velocity);
    cur_js->effort.push_back(nan(kUnavailable));  // TODO: 電流値からトルクを推定
  }

  motor_states_pub_.publish(motor_states);
  cur_js_pub_.publish(cur_js);
}

void DynamixelHandler::sendCommands()
{
  for (size_t i = 0; i < tar_js_->name.size(); ++i)
  {
    const auto& name = tar_js_->name[i];
    if (!motors_.contains(name))
    {
      rosError(name_, "Controller for joint '" << name << "' is not found.");
      continue;
    }

    const auto& cfg = motors_.at(name);
    if (cfg.operating_mode == kCurrentControlMode || cfg.operating_mode == kPwmControlMode)
    {
      rosError(name_, "Effort control is not implemented yet.");  // TODO
      continue;
    }
    else if (cfg.operating_mode == kVelocityControlMode)
    {
      if (tar_js_->velocity.size() <= i)
      {
        rosError(name_, "Unable to access index " << i << " of velocity array.");
        continue;
      }

      const int32_t cmd = tar_js_->velocity.at(i) / kVelocityDecodeFactor;
      if (pah_->write4ByteTxRx(poh_, cfg.id, kAddrGoalVelocity, cmd) < 0)
      {
        rosError(name_, "Failed to command velocity to joint '" << name << "'.");
        continue;
      }
    }
    else if (
      cfg.operating_mode == kPositionControlMode
      || cfg.operating_mode == kExtendedPositionControlMode
      || cfg.operating_mode == kCurrentBasePositionControlMode)
    {
      if (tar_js_->position.size() <= i)
      {
        rosError(name_, "Unable to access index " << i << " of position array.");
        continue;
      }

      const auto& tar_pos = tar_js_->position.at(i);
      const int32_t cmd = cfg.operating_mode == kPositionControlMode ?
                            dh_std::remap<double>(tar_pos, -M_PI, M_PI, 0, 1 << 12) :
                            tar_pos * (1 << 12) / (2 * M_PI);

      if (pah_->write4ByteTxRx(poh_, cfg.id, kAddrGoalPosition, cmd) < 0)
      {
        rosError(name_, "Failed to command position to joint '" << name << "'.");
        continue;
      }
    }
    else
    {
      rosError(name_, "Unknown operating mode: " << static_cast<int>(cfg.operating_mode));
      continue;
    }
  }
}

void DynamixelHandler::targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js)
{
  tar_js_ = tar_js;
}

void DynamixelHandler::mainTimerCb(const ros::TimerEvent& event)
{
  publishCurrentStates(event.current_real);

  if (tar_js_ != nullptr)
    sendCommands();
}
}  // namespace dynamixel_handler
