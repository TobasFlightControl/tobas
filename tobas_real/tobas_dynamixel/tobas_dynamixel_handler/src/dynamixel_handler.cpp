#include <fstream>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/unordered_map.hpp>
#include <tobas_std_tools/unordered_set.hpp>

#include <tobas_constants/constants.hpp>

#include "../include/tobas_dynamixel_handler/dynamixel_handler.hpp"
#include "../include/tobas_dynamixel_handler/constants.hpp"

using namespace std;
using namespace dynamixel;

namespace tobas_dynamixel_handler
{
DynamixelHandler::DynamixelHandler(const rclcpp::NodeOptions& options) : super(name, options)
{
  // Get ROS parameters
  getStaticRosParams();

  // Initialize Dynamixel SDK
  poh_ = PortHandler::getPortHandler(device_name_.c_str());
  pah_ = PacketHandler::getPacketHandler(protocol_version_);
  pos_sync_read_.reset(new GroupSyncRead(poh_, pah_, kAddrPresentPosition, 4));
  vel_sync_read_.reset(new GroupSyncRead(poh_, pah_, kAddrPresentVelocity, 4));
  current_sync_read_.reset(new GroupSyncRead(poh_, pah_, kAddrPresentCurrent, 2));
  pwm_sync_read_.reset(new GroupSyncRead(poh_, pah_, kAddrPresentPwm, 2));
  voltage_sync_read_.reset(new GroupSyncRead(poh_, pah_, kAddrPresentInputVoltage, 2));
  temp_sync_read_.reset(new GroupSyncRead(poh_, pah_, kAddrPresentTemperature, 1));
  hes_sync_read_.reset(new GroupSyncRead(poh_, pah_, kAddrHardwareErrorStatus, 1));
  pos_sync_write_.reset(new GroupSyncWrite(poh_, pah_, kAddrGoalPosition, 4));
  vel_sync_write_.reset(new GroupSyncWrite(poh_, pah_, kAddrGoalVelocity, 4));

  // Open serial port
  if (!poh_->openPort())
    TOBAS_EXIT("Failed to open port '", device_name_, "'");

  // Set baudrate
  if (!poh_->setBaudRate(baudrate_))
    TOBAS_EXIT("Failed to set baudrate to ", baudrate_);

  // Get motor configurations
  getMotorConfigs();

  for (const auto& [name, cfg] : motors_)
  {
    // Add parameters to GroupSyncRead objects
    if (
      !pos_sync_read_->addParam(cfg.id) || !vel_sync_read_->addParam(cfg.id) || !current_sync_read_->addParam(cfg.id)
      || !pwm_sync_read_->addParam(cfg.id) || !voltage_sync_read_->addParam(cfg.id)
      || !temp_sync_read_->addParam(cfg.id) || !hes_sync_read_->addParam(cfg.id))
      TOBAS_EXIT("Motor ID ", static_cast<int>(cfg.id), " is duplicated.");

    // Disable torque
    if (pah_->write1ByteTxRx(poh_, cfg.id, kAddrToruqeEnable, kTorqueDisable) < 0)
      TOBAS_EXIT("Failed to disable torque of '", name, "'.");

    // Set return delay time
    if (pah_->write1ByteTxRx(poh_, cfg.id, kAddrReturnDelayTime, return_delay_time_) < 0)
      TOBAS_EXIT("Failed to set return delay time of '", name, "'.");

    // Set operating mode
    if (pah_->write1ByteTxRx(poh_, cfg.id, kAddrOperatingMode, cfg.operating_mode) < 0)
      TOBAS_EXIT("Failed to set operating mode of '", name, "'.");

    // Enable torque
    if (pah_->write1ByteTxRx(poh_, cfg.id, kAddrToruqeEnable, kTorqueEnable) < 0)
      TOBAS_EXIT("Failed to enable torque of '", name, "'.");

    TOBAS_INFO("'", name, "' is initialized.");
  }

  // Reduce latency
  if (!setMinimumLatency())
    TOBAS_EXIT("Failed to set communication latency.");

  // Register publishers and subscribers
  registerPublishers();
  registerSubscribers();

  // Register service servers
  enable_torques_ss_ = createService(kEnableTorquesSrv, &self::enableTorquesServiceCb, this);

  // Start main timer with maximum rate
  main_timer_ = createTimer(rclcpp::Duration(0), &self::mainTimerCb, this);
}

DynamixelHandler::~DynamixelHandler()
{
  // Disable torque
  disableTorques();

  // Close serial port
  poh_->closePort();
}

void DynamixelHandler::getStaticRosParams()
{
  ros2::getParam(pnh_, "joint_names", jnt_names_);
  ros2::getParam(pnh_, "device_name", device_name_, string(kDefaultDeviceName));
  ros2::getParam(pnh_, "protocol_version", protocol_version_, kDefaultProtocolVersion);
  ros2::getParam(pnh_, "baudrate", baudrate_, kDefaultBaudRate);
  ros2::getParam(pnh_, "return_delay_time", return_delay_time_, kDefaultReturnDelayTime);
  ros2::getParam(pnh_, "read_pwm", read_pwm_, kDefaultReadPwm);
  ros2::getParam(pnh_, "read_current", read_current_, kDefaultReadCurrent);
  ros2::getParam(pnh_, "read_velocity", read_velocity_, kDefaultReadVelocity);
  ros2::getParam(pnh_, "read_position", read_position_, kDefaultReadPosition);
  ros2::getParam(pnh_, "read_voltage", read_voltage_, kDefaultReadVoltage);
  ros2::getParam(pnh_, "read_temperature", read_temperature_, kDefaultReadTemperature);
}

void DynamixelHandler::registerPublishers()
{
  motor_states_pub_ = createPublisher<tobas_dynamixel_msgs::MotorStateArray>(kMotorStatesTopic);
}

void DynamixelHandler::registerSubscribers()
{
  event_sub_ = createSubscriber(tobas::kEventTopic, &self::eventCb, this);
  positions_sub_ = createSubscriber(kJointPositionsCmdTopic, &self::jointPositionsCmdCb, this);
  velocities_sub_ = createSubscriber(kJointVelocitiesCmdTopic, &self::jointVelocitiesCmdCb, this);
  efforts_sub_ = createSubscriber(kJointEffortsCmdTopic, &self::jointEffortsCmdCb, this);
}

bool DynamixelHandler::setMinimumLatency()
{
  const auto pos = device_name_.rfind('/');
  const auto port = pos == string::npos ? device_name_ : device_name_.substr(pos + 1);
  const auto file_path = "/sys/bus/usb-serial/devices/" + port + "/latency_timer";

  ofstream file(file_path);
  if (!file.is_open())
    return false;

  file << kMinimumLatency;  // uint8だと反映されなかった
  file.close();

  TOBAS_INFO("Communication latency is updated successfully.");
  return true;
}

void DynamixelHandler::getMotorConfigs()
{
  DynamixelConfig cfg;
  string operating_mode;
  unordered_set<uint8_t> used_ids;
  uint16_t model_number;
  uint8_t temp_limit;
  uint16_t max_voltage_limit;
  uint16_t min_voltage_limit;
  uint16_t pwm_limit;
  uint16_t current_limit;
  uint32_t acc_limit;
  uint32_t vel_limit;
  uint32_t max_pos_limit;
  uint32_t min_pos_limit;

  for (const auto& name : jnt_names_)
  {
    // ID
    ros2::getParam(pnh_, name + "/id", cfg.id, kDefaultId);
    if (tobas_std::contains(used_ids, cfg.id))
      TOBAS_EXIT("Motor ID ", static_cast<int>(cfg.id), " is duplicated.");
    used_ids.insert(cfg.id);

    // Current scaling factor
    if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrModelNumber, &model_number) < 0)
      TOBAS_EXIT("Failed to get model number of '", name, "'.");
    switch (model_number)
    {
      case kModelNumberXL430W250:
        cfg.current_available = false;
        break;
      case kModelNumberXC430W250:
        cfg.current_available = false;
        break;
      case kModelNumberXM430W350:
        cfg.current_available = true;
        cfg.current_scaling_factor = 2.69 / 1000;
        break;
      default:
        TOBAS_EXIT("Setting for model number ", model_number, " is not implemented yet.");
    }

    // Operating mode
    ros2::getParam(pnh_, name + "/operating_mode", operating_mode, string(kDefaultOperatingMode));
    if (operating_mode == "current")
    {
      if (!cfg.current_available)
        TOBAS_EXIT("Current control mode is unavailable for '", name, "'.");
      cfg.operating_mode = kControlModePosition;
    }
    else if (operating_mode == "velocity")
      cfg.operating_mode = kControlModeVelocity;
    else if (operating_mode == "position")
      cfg.operating_mode = kControlModePosition;
    else if (operating_mode == "extended_position")
      cfg.operating_mode = kControlModeExtendedPosition;
    else if (operating_mode == "current_base_position")
    {
      if (!cfg.current_available)
        TOBAS_EXIT("Current-base position control mode is unavailable for '", name, "'.");
      cfg.operating_mode = kControlModeCurrentBasePosition;
    }
    else if (operating_mode == "pwm")
      cfg.operating_mode = kControlModePwm;
    else
      TOBAS_EXIT("Unknown operating mode for '", name, "'.");

    // Limits
    if (pah_->read1ByteTxRx(poh_, cfg.id, kAddrTemperatureLimit, &temp_limit) == 0)
      cfg.temp_limit = static_cast<double>(temp_limit) * kDecodeFactorTemp;
    else
      TOBAS_EXIT("Failed to get temperature limit of '", name, "'.");

    if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrMaxVoltageLimit, &max_voltage_limit) == 0)
      cfg.voltage_limit.upper = static_cast<double>(max_voltage_limit) * kDecodeFactorVoltage;
    else
      TOBAS_EXIT("Failed to get maximum voltage limit of '", name, "'.");

    if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrMinVoltageLimit, &min_voltage_limit) == 0)
      cfg.voltage_limit.lower = static_cast<double>(min_voltage_limit) * kDecodeFactorVoltage;
    else
      TOBAS_EXIT("Failed to get minimum voltage limit of '", name, "'.");

    if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrPwmLimit, &pwm_limit) == 0)
      cfg.pwm_limit = static_cast<double>(pwm_limit) * kDecodeFactorPwm;
    else
      TOBAS_EXIT("Failed to get PWM limit of '", name, "'.");

    if (cfg.current_available)
    {
      if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrCurrentLimit, &current_limit) == 0)
        cfg.current_limit = static_cast<double>(current_limit) * cfg.current_scaling_factor;
      else
        TOBAS_EXIT("Failed to get current limit of '", name, "'.");
    }
    else
    {
      cfg.current_limit = nan(kUnavailable);
    }

    if (pah_->read4ByteTxRx(poh_, cfg.id, kAddrAccelerationLimit, &acc_limit) == 0)
      cfg.acc_limit = static_cast<double>(acc_limit) * kDecodeFactorAcc;
    else
      TOBAS_EXIT("Failed to get acceleration limit of '", name, "'.");

    if (pah_->read4ByteTxRx(poh_, cfg.id, kAddrVelocityLimit, &vel_limit) == 0)
      cfg.vel_limit = static_cast<double>(vel_limit) * kDecodeFactorVel;
    else
      TOBAS_EXIT("Failed to get velocity limit of '", name, "'.");

    if (pah_->read4ByteTxRx(poh_, cfg.id, kAddrMaxPositionLimit, &max_pos_limit) == 0)
      cfg.pos_limit.upper = math::remap<double>(max_pos_limit, 0, 1 << 12, -M_PI, M_PI);
    else
      TOBAS_EXIT("Failed to get maximum position limit of '", name, "'.");

    if (pah_->read4ByteTxRx(poh_, cfg.id, kAddrMinPositionLimit, &min_pos_limit) == 0)
      cfg.pos_limit.lower = math::remap<double>(min_pos_limit, 0, 1 << 12, -M_PI, M_PI);
    else
      TOBAS_EXIT("Failed to get minimum position limit of '", name, "'.");

    // Insert motor config
    motors_[name] = cfg;
  }
}

bool DynamixelHandler::enableTorques()
{
  for (const auto& [name, cfg] : motors_)
  {
    if (pah_->write1ByteTxRx(poh_, cfg.id, kAddrToruqeEnable, kTorqueEnable) < 0)
    {
      TOBAS_ERROR("Failed to enable torque of '", name, "'.");
      return false;
    }
  }

  is_enabled_ = true;
  return true;
}

bool DynamixelHandler::disableTorques()
{
  for (const auto& [name, cfg] : motors_)
  {
    if (pah_->write1ByteTxRx(poh_, cfg.id, kAddrToruqeEnable, kTorqueDisable) < 0)
    {
      TOBAS_ERROR("Failed to disable torque of '", name, "'.");
      return false;
    }
  }

  is_enabled_ = false;
  return true;
}

void DynamixelHandler::printHardwareErrorStatus()
{
  if (hes_sync_read_->txRxPacket() < 0)
  {
    TOBAS_ERROR("Failed to receive a sync packet of hardware error status.");
    return;
  }

  for (const auto& [name, cfg] : motors_)
  {
    const uint8_t hes = hes_sync_read_->getData(cfg.id, kAddrHardwareErrorStatus, 1);
    if (hes & kErrorInputVoltage)
      TOBAS_ERROR("Input voltage error in '", name, "'");
    if (hes & kErrorHallSensor)
      TOBAS_ERROR("Hall sensor error in '", name, "'");
    if (hes & kErrorOverheating)
      TOBAS_ERROR("Overheating error in '", name, "'");
    if (hes & kErrorMotorEncoder)
      TOBAS_ERROR("Motor encoder error in '", name, "'");
    if (hes & kErrorElectricalShock)
      TOBAS_ERROR("Electrical shock error in '", name, "'");
    if (hes & kErrorOverload)
      TOBAS_ERROR("Overload error in '", name, "'");
  }
}

void DynamixelHandler::publishCurrentStates(const rclcpp::Time& cur_time)
{
  // Create motor states message
  const auto motor_states = std::make_unique<tobas_dynamixel_msgs::MotorStateArray>();
  motor_states->header.stamp = cur_time;

  // Read packets
  if (read_position_ && pos_sync_read_->txRxPacket() < 0)
  {
    TOBAS_ERROR("Failed to receive a sync packet of present position. Disabling torques.");
    disableTorques();
    printHardwareErrorStatus();  // FIXME: モータのシャットダウン後はHESの取得もできない
    return;
  }
  if (read_velocity_ && vel_sync_read_->txRxPacket() < 0)
  {
    TOBAS_ERROR("Failed to receive a sync packet of present velocity. Disabling torques.");
    disableTorques();
    printHardwareErrorStatus();
    return;
  }
  if (read_current_ && current_sync_read_->txRxPacket() < 0)
  {
    TOBAS_ERROR("Failed to receive a sync packet of present current. Disabling torques.");
    disableTorques();
    printHardwareErrorStatus();
    return;
  }
  if (read_pwm_ && pwm_sync_read_->txRxPacket() < 0)
  {
    TOBAS_ERROR("Failed to receive a sync packet of present PWM. Disabling torques.");
    disableTorques();
    printHardwareErrorStatus();
    return;
  }
  if (read_voltage_ && voltage_sync_read_->txRxPacket() < 0)
  {
    TOBAS_ERROR("Failed to receive a sync packet of present input voltage. Disabling torques.");
    disableTorques();
    printHardwareErrorStatus();
    return;
  }
  if (read_temperature_ && temp_sync_read_->txRxPacket() < 0)
  {
    TOBAS_ERROR("Failed to receive a sync packet of present temperature. Disabling torques.");
    disableTorques();
    printHardwareErrorStatus();
    return;
  }

  // Compute joint states in the SI unit system
  for (const auto& [name, cfg] : motors_)
  {
    motor_state_.name = name;

    if (read_position_)
    {
      const int32_t pos_raw = pos_sync_read_->getData(cfg.id, kAddrPresentPosition, 4);
      motor_state_.position = math::remap<double>(pos_raw, 0, 1 << 12, -M_PI, M_PI);
    }
    else
    {
      motor_state_.position = nan(kInactive);
    }

    if (read_velocity_)
    {
      const int32_t vel_raw = vel_sync_read_->getData(cfg.id, kAddrPresentVelocity, 4);
      motor_state_.velocity = static_cast<double>(vel_raw) * kDecodeFactorVel;
    }
    else
    {
      motor_state_.velocity = nan(kInactive);
    }

    if (read_current_)
    {
      const int16_t current_raw = current_sync_read_->getData(cfg.id, kAddrPresentCurrent, 2);
      if (cfg.current_available)
      {
        motor_state_.current = static_cast<double>(current_raw) * cfg.current_scaling_factor;
        motor_state_.load = nan(kUnavailable);
      }
      else
      {
        motor_state_.current = nan(kUnavailable);
        motor_state_.load = static_cast<double>(current_raw) * 0.1;
      }
    }
    else
    {
      motor_state_.current = nan(kInactive);
      motor_state_.load = nan(kInactive);
    }

    if (read_pwm_)
    {
      const int16_t pwm_raw = pwm_sync_read_->getData(cfg.id, kAddrPresentPwm, 2);
      motor_state_.pwm = static_cast<double>(pwm_raw) * kDecodeFactorPwm;
    }
    else
    {
      motor_state_.pwm = nan(kInactive);
    }

    if (read_voltage_)
    {
      const uint16_t voltage_yaw = voltage_sync_read_->getData(cfg.id, kAddrPresentInputVoltage, 2);
      motor_state_.input_voltage = static_cast<double>(voltage_yaw) * kDecodeFactorVoltage;
    }
    else
    {
      motor_state_.input_voltage = nan(kInactive);
    }

    if (read_temperature_)
    {
      const uint8_t temp_raw = temp_sync_read_->getData(cfg.id, kAddrPresentTemperature, 1);
      motor_state_.temperature = static_cast<double>(temp_raw) * kDecodeFactorTemp;
    }
    else
    {
      motor_state_.temperature = nan(kInactive);
    }

    motor_states->states.push_back(motor_state_);
  }

  // Publish motor states message
  motor_states_pub_->publish(motor_states);
}

void DynamixelHandler::eventCb(const tobas_msgs::msg::Event::ConstSharedPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::msg::Event::SYSTEM_CRITICAL:
      TOBAS_WARN("System critical event message is received. Disabling torques.");
      disableTorques();
      break;
    default:
      break;
  }
}

void DynamixelHandler::jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions)
{
  if (!is_enabled_)
  {
    TOBAS_ERROR("Motors are disabled. You cannot command positions.");
    return;
  }

  const auto size = positions->commands.size();
  goal_positions_.resize(size);
  pos_sync_write_->clearParam();

  for (size_t i = 0; i < size; ++i)
  {
    const auto& jnt_name = positions->commands[i].name;
    if (!tobas_std::contains(motors_, jnt_name))
    {
      TOBAS_ERROR("Controller for joint '", jnt_name, "' is not found.");
      continue;
    }

    const auto& cfg = motors_.at(jnt_name);
    auto tar_pos = positions->commands[i].data;
    if (cfg.operating_mode == kControlModePosition)
    {
      if (cfg.pos_limit.clamp(tar_pos, tar_pos))
        TOBAS_WARN("Target position of joint '", jnt_name, "' is out of limit. The value is clamped to ", tar_pos);
      goal_positions_[i] = math::remap<double>(tar_pos, -M_PI, M_PI, 0, 1 << 12);
    }
    else if (cfg.operating_mode == kControlModeExtendedPosition || cfg.operating_mode == kControlModeCurrentBasePosition)
    {
      goal_positions_[i] = tar_pos / kDecodeFactorPos;
    }
    else
    {
      TOBAS_ERROR("The operating mode of joint '", jnt_name, "' is not position.");
      continue;
    }

    if (!pos_sync_write_->addParam(cfg.id, (uint8_t*)&goal_positions_[i]))
      TOBAS_ERROR("Failed to set goal position of joint '", jnt_name, "'.");
  }

  if (pos_sync_write_->txPacket() < 0)
    TOBAS_ERROR("Failed to transmit a sync write instruction packet of positions.");
}

void DynamixelHandler::jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities)
{
  if (!is_enabled_)
  {
    TOBAS_ERROR("Motors are disabled. You cannot command velocities.");
    return;
  }

  const auto size = velocities->commands.size();
  goal_velocities_.resize(size);
  vel_sync_write_->clearParam();

  for (size_t i = 0; i < size; ++i)
  {
    const auto& jnt_name = velocities->commands[i].name;
    if (!tobas_std::contains(motors_, jnt_name))
    {
      TOBAS_ERROR("Controller for joint '", jnt_name, "' is not found.");
      continue;
    }

    const auto& cfg = motors_.at(jnt_name);
    if (cfg.operating_mode != kControlModeVelocity)
    {
      TOBAS_ERROR("The operating mode of joint '", jnt_name, "' is not velocity.");
      continue;
    }

    auto tar_vel = velocities->commands[i].data;
    if (abs(tar_vel) > cfg.vel_limit)
    {
      tar_vel = clamp(tar_vel, -cfg.vel_limit, cfg.vel_limit);
      TOBAS_WARN("Target velocity of joint '", jnt_name, "' is out of limit. The value is clamped to ", tar_vel);
    }

    goal_velocities_[i] = tar_vel / kDecodeFactorVel;
    if (!vel_sync_write_->addParam(cfg.id, (uint8_t*)&goal_velocities_[i]))
      TOBAS_ERROR("Failed to set goal velocity of joint '", jnt_name, "'.");
  }

  if (vel_sync_write_->txPacket() < 0)
    TOBAS_ERROR("Failed to transmit a sync write instruction packet of velocities.");
}

void DynamixelHandler::jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts)
{
  if (!is_enabled_)
  {
    TOBAS_ERROR("Motors are disabled. You cannot command efforts.");
    return;
  }

  const auto size = efforts->commands.size();

  for (size_t i = 0; i < size; ++i)
  {
    const auto& jnt_name = efforts->commands[i].name;
    if (!tobas_std::contains(motors_, jnt_name))
    {
      TOBAS_ERROR("Controller for joint '", jnt_name, "' is not found.");
      continue;
    }

    const auto& cfg = motors_.at(jnt_name);
    if (cfg.operating_mode != kControlModeCurrent && cfg.operating_mode != kControlModePwm)
    {
      TOBAS_ERROR("The operating mode of joint '", jnt_name, "' is not effort.");
      continue;
    }

    TOBAS_ERROR("Effort control is not implemented yet.");  // TODO
  }
}

bool DynamixelHandler::enableTorquesServiceCb(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
{
  if (req.data)
  {
    if (enableTorques())
    {
      res.success = true;
      res.message = "Torques enabled.";
    }
    else
    {
      res.success = false;
      res.message = "Failed to enable torques.";
    }
  }
  else
  {
    if (disableTorques())
    {
      res.success = true;
      res.message = "Torques disabled.";
    }
    else
    {
      res.success = false;
      res.message = "Failed to disable torques.";
    }
  }

  return true;
}

void DynamixelHandler::mainTimerCb()
{
  if (!is_enabled_)
    return;

  publishCurrentStates(get_clock()->now());
}
}  // namespace tobas_dynamixel_handler
