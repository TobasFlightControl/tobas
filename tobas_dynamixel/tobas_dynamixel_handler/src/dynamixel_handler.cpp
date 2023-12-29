#include <fstream>

#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/unix.hpp>
#include <tobas_std_tools/unordered_map.hpp>
#include <tobas_std_tools/unordered_set.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_dynamixel_handler/dynamixel_handler.hpp"
#include "../include/tobas_dynamixel_handler/constants.hpp"

using namespace std;

namespace tobas_dynamixel_handler
{
DynamixelHandler::DynamixelHandler(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
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

    // Set return delay time
    if (pah_->write1ByteTxRx(poh_, cfg.id, kAddrReturnDelayTime, return_delay_time_) < 0)
      rosError(name_, "Failed to set return delay time of '" << name << "'.");

    // Set operating mode
    if (pah_->write1ByteTxRx(poh_, cfg.id, kAddrOperatingMode, cfg.operating_mode) < 0)
      ROS_THROW_NAMED(name_, "Failed to set operating mode of '" << name << "'.");

    // Enable torque
    if (pah_->write1ByteTxRx(poh_, cfg.id, kAddrToruqeEnable, kTorqueEnable) < 0)
      ROS_THROW_NAMED(name_, "Failed to enable torque of '" << name << "'.");

    rosInfo(name_, "'" << name << "' is initialized.");
  }

  // Reduce latency
  if (tobas_std::isSuperUser())
    setMinimumLatency();
  else
    rosWarn(name_, "Please execute with root privileges to set minimum communication latency.");

  // Register publishers and subscribers
  registerPublishers();
  registerSubscribers();

  // Start main timer with maximum rate
  main_timer_ = nh_.createTimer(ros::Duration(0), &self::mainTimerCb, this);
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
  tobas_ros::getParam(pnh_, "joint_names", jnt_names_);
  tobas_ros::getParam(pnh_, "device_name", device_name_, string(kDefaultDeviceName));
  tobas_ros::getParam(pnh_, "protocol_version", protocol_version_, kDefaultProtocolVersion);
  tobas_ros::getParam(pnh_, "baudrate", baudrate_, kDefaultBaudRate);
  tobas_ros::getParam(pnh_, "return_delay_time", return_delay_time_, kDefaultReturnDelayTime);
  tobas_ros::getParam(pnh_, "read_pwm", read_pwm_, kDefaultReadPwm);
  tobas_ros::getParam(pnh_, "read_current", read_current_, kDefaultReadCurrent);
  tobas_ros::getParam(pnh_, "read_velocity", read_velocity_, kDefaultReadVelocity);
  tobas_ros::getParam(pnh_, "read_position", read_position_, kDefaultReadPosition);
  tobas_ros::getParam(pnh_, "read_voltage", read_voltage_, kDefaultReadVoltage);
  tobas_ros::getParam(pnh_, "read_temperature", read_temperature_, kDefaultReadTemperature);
}

void DynamixelHandler::setMinimumLatency()
{
  const auto pos = device_name_.rfind('/');
  const auto port = pos == string::npos ? device_name_ : device_name_.substr(pos + 1);
  const auto file_path = "/sys/bus/usb-serial/devices/" + port + "/latency_timer";
  ofstream file(file_path);
  if (file.is_open())
  {
    file << kMinimumLatency;  // uint8だと反映されなかった
    file.close();
    rosInfo(name_, "Communication latency is set to " << kMinimumLatency);
  }
  else
  {
    rosError(
      name_, "Unable to open file '" << file_path << "'. Communication latency is unchanged.");
  }
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
    tobas_ros::getParam(pnh_, name + "/id", cfg.id, kDefaultId);
    if (tobas_std::contains(used_ids, cfg.id))
    {
      rosError(name_, "Motor ID " << static_cast<int>(cfg.id) << " is duplicated.");
      continue;
    }
    used_ids.insert(cfg.id);

    // Current scaling factor
    if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrModelNumber, &model_number) < 0)
    {
      rosError(name_, "Failed to get model number of '" << name << "'.");
      continue;
    }
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
        rosWarn(name_, "Setting for model number " << model_number << " is not implemented yet.");
        cfg.current_available = false;
        break;
    }

    // Operating mode
    tobas_ros::getParam(
      pnh_, name + "/operating_mode", operating_mode, string(kDefaultOperatingMode));
    if (operating_mode == "current")
    {
      if (!cfg.current_available)
      {
        rosError(name_, "Current control mode is unavailable for '" << name << "'.");
        continue;
      }
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
      {
        rosError(name_, "Current-base position control mode is unavailable for '" << name << "'.");
        continue;
      }
      cfg.operating_mode = kControlModeCurrentBasePosition;
    }
    else if (operating_mode == "pwm")
    {
      cfg.operating_mode = kControlModePwm;
    }
    else
    {
      rosError(name_, "Unknown operating mode for '" << name << "'.");
      continue;
    }

    // Limits
    if (pah_->read1ByteTxRx(poh_, cfg.id, kAddrTemperatureLimit, &temp_limit) == 0)
    {
      cfg.temp_limit = static_cast<double>(temp_limit) * kDecodeFactorTemp;
    }
    else
    {
      rosError(name_, "Failed to get temperature limit of '" << name << "'.");
      continue;
    }

    if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrMaxVoltageLimit, &max_voltage_limit) == 0)
    {
      cfg.voltage_limit.upper = static_cast<double>(max_voltage_limit) * kDecodeFactorVoltage;
    }
    else
    {
      rosError(name_, "Failed to get maximum voltage limit of '" << name << "'.");
      continue;
    }

    if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrMinVoltageLimit, &min_voltage_limit) == 0)
    {
      cfg.voltage_limit.lower = static_cast<double>(min_voltage_limit) * kDecodeFactorVoltage;
    }
    else
    {
      rosError(name_, "Failed to get minimum voltage limit of '" << name << "'.");
      continue;
    }

    if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrPwmLimit, &pwm_limit) == 0)
    {
      cfg.pwm_limit = static_cast<double>(pwm_limit) * kDecodeFactorPwm;
    }
    else
    {
      rosError(name_, "Failed to get PWM limit of '" << name << "'.");
      continue;
    }

    if (cfg.current_available)
    {
      if (pah_->read2ByteTxRx(poh_, cfg.id, kAddrCurrentLimit, &current_limit) == 0)
      {
        cfg.current_limit = static_cast<double>(current_limit) * cfg.current_scaling_factor;
      }
      else
      {
        rosError(name_, "Failed to get current limit of '" << name << "'.");
        continue;
      }
    }
    else
    {
      cfg.current_limit = nan(kUnavailable);
    }

    if (pah_->read4ByteTxRx(poh_, cfg.id, kAddrAccelerationLimit, &acc_limit) == 0)
    {
      cfg.acc_limit = static_cast<double>(acc_limit) * kDecodeFactorAcc;
    }
    else
    {
      rosError(name_, "Failed to get acceleration limit of '" << name << "'.");
      continue;
    }

    if (pah_->read4ByteTxRx(poh_, cfg.id, kAddrVelocityLimit, &vel_limit) == 0)
    {
      cfg.vel_limit = static_cast<double>(vel_limit) * kDecodeFactorVel;
    }
    else
    {
      rosError(name_, "Failed to get velocity limit of '" << name << "'.");
      continue;
    }

    if (pah_->read4ByteTxRx(poh_, cfg.id, kAddrMaxPositionLimit, &max_pos_limit) == 0)
    {
      cfg.pos_limit.upper = tobas_std::remap<double>(max_pos_limit, 0, 1 << 12, -M_PI, M_PI);
    }
    else
    {
      rosError(name_, "Failed to get maximum position limit of '" << name << "'.");
      continue;
    }

    if (pah_->read4ByteTxRx(poh_, cfg.id, kAddrMinPositionLimit, &min_pos_limit) == 0)
    {
      cfg.pos_limit.lower = tobas_std::remap<double>(min_pos_limit, 0, 1 << 12, -M_PI, M_PI);
    }
    else
    {
      rosError(name_, "Failed to get minimum position limit of '" << name << "'.");
      continue;
    }

    // Insert motor config
    motors_[name] = cfg;
  }
}

void DynamixelHandler::registerPublishers()
{
  motor_states_pub_ = nh_.advertise<tobas_dynamixel_msgs::MotorStates>("motor_states", 1);
}

void DynamixelHandler::registerSubscribers()
{
  positions_sub_ =
    nh_.subscribe(kJointPositionsCmdTopic, 1, &self::jointPositionsCmdCb, this, tcpNoDelay());
  velocities_sub_ =
    nh_.subscribe(kJointVelocitiesCmdTopic, 1, &self::jointVelocitiesCmdCb, this, tcpNoDelay());
  efforts_sub_ =
    nh_.subscribe(kJointEffortsCmdTopic, 1, &self::jointEffortsCmdCb, this, tcpNoDelay());
}

void DynamixelHandler::publishCurrentStates(const ros::Time& cur_time)
{
  // Create motor states message
  const auto motor_states = boost::make_shared<tobas_dynamixel_msgs::MotorStates>();
  motor_states->header.stamp = cur_time;

  // Read packets
  dynamixel::GroupSyncRead pos_sync_read(poh_, pah_, kAddrPresentPosition, 4);
  dynamixel::GroupSyncRead vel_sync_read(poh_, pah_, kAddrPresentVelocity, 4);
  dynamixel::GroupSyncRead current_sync_read(poh_, pah_, kAddrPresentCurrent, 2);
  dynamixel::GroupSyncRead pwm_sync_read(poh_, pah_, kAddrPresentPwm, 2);
  dynamixel::GroupSyncRead voltage_sync_read(poh_, pah_, kAddrPresentInputVoltage, 2);
  dynamixel::GroupSyncRead temp_sync_read(poh_, pah_, kAddrPresentTemperature, 1);
  if (read_position_ && readSyncPacket(pos_sync_read) < 0)
    rosError(name_, "Failed to receive a sync packet of present position.");
  if (read_velocity_ && readSyncPacket(vel_sync_read) < 0)
    rosError(name_, "Failed to receive a sync packet of present velocity.");
  if (read_current_ && readSyncPacket(current_sync_read) < 0)
    rosError(name_, "Failed to receive a sync packet of present current.");
  if (read_pwm_ && readSyncPacket(pwm_sync_read) < 0)
    rosError(name_, "Failed to receive a sync packet of present PWM.");
  if (read_voltage_ && readSyncPacket(voltage_sync_read) < 0)
    rosError(name_, "Failed to receive a sync packet of present input voltage.");
  if (read_temperature_ && readSyncPacket(temp_sync_read) < 0)
    rosError(name_, "Failed to receive a sync packet of present temperature.");

  // Compute joint states in the SI unit system
  for (const auto& [name, cfg] : motors_)
  {
    if (read_position_)
    {
      const int32_t pos_raw = pos_sync_read.getData(cfg.id, kAddrPresentPosition, 4);
      motor_state_.position = tobas_std::remap<double>(pos_raw, 0, 1 << 12, -M_PI, M_PI);
    }
    else
    {
      motor_state_.position = nan(kInactive);
    }

    if (read_velocity_)
    {
      const int32_t vel_raw = vel_sync_read.getData(cfg.id, kAddrPresentVelocity, 4);
      motor_state_.velocity = static_cast<double>(vel_raw) * kDecodeFactorVel;
    }
    else
    {
      motor_state_.velocity = nan(kInactive);
    }

    if (read_current_)
    {
      const int16_t current_raw = current_sync_read.getData(cfg.id, kAddrPresentCurrent, 2);
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
      const int16_t pwm_raw = pos_sync_read.getData(cfg.id, kAddrPresentPwm, 2);
      motor_state_.pwm = static_cast<double>(pwm_raw) * kDecodeFactorPwm;
    }
    else
    {
      motor_state_.pwm = nan(kInactive);
    }

    if (read_voltage_)
    {
      const uint16_t voltage_yaw = voltage_sync_read.getData(cfg.id, kAddrPresentInputVoltage, 2);
      motor_state_.input_voltage = static_cast<double>(voltage_yaw) * kDecodeFactorVoltage;
    }
    else
    {
      motor_state_.input_voltage = nan(kInactive);
    }

    if (read_temperature_)
    {
      const uint8_t temp_raw = temp_sync_read.getData(cfg.id, kAddrPresentTemperature, 1);
      motor_state_.temperature = static_cast<double>(temp_raw) * kDecodeFactorTemp;
    }
    else
    {
      motor_state_.temperature = nan(kInactive);
    }

    motor_states->names.push_back(name);
    motor_states->states.push_back(motor_state_);
  }

  // Publish motor states message
  motor_states_pub_.publish(motor_states);
}

int DynamixelHandler::readSyncPacket(dynamixel::GroupSyncRead& sync_read)
{
  // sync_read.clearParam();

  for (const auto& [jnt_name, cfg] : motors_)
  {
    if (!sync_read.addParam(cfg.id))
      rosError(name_, "Motor ID " << static_cast<int>(cfg.id) << " is duplicated.");
  }

  return sync_read.txRxPacket();
}

void DynamixelHandler::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void DynamixelHandler::jointPositionsCmdCb(const tobas_msgs::JointPositionsConstPtr& positions)
{
  const auto size = positions->name.size();

  if (positions->data.size() != size)
  {
    rosError(name_, "The sizes of name and data in joint positions message do not match.");
    return;
  }

  vector<int32_t> goal_positions(size);
  dynamixel::GroupSyncWrite pos_sync_write(poh_, pah_, kAddrGoalPosition, 4);

  for (size_t i = 0; i < size; ++i)
  {
    const auto& jnt_name = positions->name[i];
    if (!tobas_std::contains(motors_, jnt_name))
    {
      rosError(name_, "Controller for joint '" << jnt_name << "' is not found.");
      continue;
    }

    const auto& cfg = motors_.at(jnt_name);
    auto tar_pos = positions->data[i];
    if (cfg.operating_mode == kControlModePosition)
    {
      if (cfg.pos_limit.clamp(tar_pos, tar_pos))
        rosWarn(
          name_, "Target position of joint '"
                   << jnt_name << "' is out of limit. The value is clamped to " << tar_pos);
      goal_positions[i] = tobas_std::remap<double>(tar_pos, -M_PI, M_PI, 0, 1 << 12);
    }
    else if (
      cfg.operating_mode == kControlModeExtendedPosition
      || cfg.operating_mode == kControlModeCurrentBasePosition)
    {
      goal_positions[i] = tar_pos / kDecodeFactorPos;
    }
    else
    {
      rosError(name_, "The operating mode of joint '" << jnt_name << "' is not position.");
      continue;
    }

    if (!pos_sync_write.addParam(cfg.id, (uint8_t*)&goal_positions[i]))
      rosError(name_, "Failed to set goal position of joint '" << jnt_name << "'.");
  }

  if (pos_sync_write.txPacket() < 0)
    rosError(name_, "Failed to transmit a sync write instruction packet of positions.");
}

void DynamixelHandler::jointVelocitiesCmdCb(const tobas_msgs::JointVelocitiesConstPtr& velocities)
{
  const auto size = velocities->name.size();

  if (velocities->data.size() != size)
  {
    rosError(name_, "The sizes of name and data in joint velocities message do not match.");
    return;
  }

  vector<int32_t> goal_velocities(size);
  dynamixel::GroupSyncWrite vel_sync_write(poh_, pah_, kAddrGoalVelocity, 4);

  for (size_t i = 0; i < size; ++i)
  {
    const auto& jnt_name = velocities->name[i];
    if (!tobas_std::contains(motors_, jnt_name))
    {
      rosError(name_, "Controller for joint '" << jnt_name << "' is not found.");
      continue;
    }

    const auto& cfg = motors_.at(jnt_name);
    if (cfg.operating_mode != kControlModeVelocity)
    {
      rosError(name_, "The operating mode of joint '" << jnt_name << "' is not velocity.");
      continue;
    }

    auto tar_vel = velocities->data[i];
    if (abs(tar_vel) > cfg.vel_limit)
    {
      tar_vel = clamp(tar_vel, -cfg.vel_limit, cfg.vel_limit);
      rosWarn(
        name_, "Target velocity of joint '"
                 << jnt_name << "' is out of limit. The value is clamped to " << tar_vel);
    }

    goal_velocities[i] = tar_vel / kDecodeFactorVel;
    if (!vel_sync_write.addParam(cfg.id, (uint8_t*)&goal_velocities[i]))
      rosError(name_, "Failed to set goal velocity of joint '" << jnt_name << "'.");
  }

  if (vel_sync_write.txPacket() < 0)
    rosError(name_, "Failed to transmit a sync write instruction packet of velocities.");
}

void DynamixelHandler::jointEffortsCmdCb(const tobas_msgs::JointEffortsConstPtr& efforts)
{
  const auto size = efforts->name.size();

  if (efforts->data.size() != size)
  {
    rosError(name_, "The sizes of name and data in joint efforts message do not match.");
    return;
  }

  for (size_t i = 0; i < size; ++i)
  {
    const auto& jnt_name = efforts->name[i];

    if (!tobas_std::contains(motors_, jnt_name))
    {
      rosError(name_, "Controller for joint '" << jnt_name << "' is not found.");
      continue;
    }

    const auto& cfg = motors_.at(jnt_name);

    if (cfg.operating_mode != kControlModeCurrent && cfg.operating_mode != kControlModePwm)
    {
      rosError(name_, "The operating mode of joint '" << jnt_name << "' is not effort.");
      continue;
    }

    rosError(name_, "Effort control is not implemented yet.");  // TODO
  }
}

void DynamixelHandler::mainTimerCb(const ros::TimerEvent& event)
{
  publishCurrentStates(event.current_real);
}
}  // namespace tobas_dynamixel_handler
