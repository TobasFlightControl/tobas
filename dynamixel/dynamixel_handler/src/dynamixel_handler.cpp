#include <fstream>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/unix.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/dynamixel_handler/dynamixel_handler.hpp"
#include "../include/dynamixel_handler/constants.hpp"

using namespace std;

namespace dynamixel_handler
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
  if (dh_std::isSuperUser())
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
  dh_ros::getParam(pnh_, "joint_names", jnt_names_);
  dh_ros::getParam(pnh_, "device_name", device_name_, string(kDefaultDeviceName));
  dh_ros::getParam(pnh_, "protocol_version", protocol_version_, kDefaultProtocolVersion);
  dh_ros::getParam(pnh_, "baudrate", baudrate_, kDefaultBaudRate);
  dh_ros::getParam(pnh_, "return_delay_time", return_delay_time_, kDefaultReturnDelayTime);
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

void DynamixelHandler::registerPublishers()
{
  motor_states_pub_ = nh_.advertise<dynamixel_msgs::MotorStates>("motor_states", 1);
  cur_js_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void DynamixelHandler::registerSubscribers()
{
  positions_sub_ = nh_.subscribe(
    tobas::kJointPositionsCmdTopic, 1, &self::jointPositionsCmdCb, this, tcpNoDelay());
  velocities_sub_ = nh_.subscribe(
    tobas::kJointVelocitiesCmdTopic, 1, &self::jointVelocitiesCmdCb, this, tcpNoDelay());
  efforts_sub_ =
    nh_.subscribe(tobas::kJointEffortsCmdTopic, 1, &self::jointEffortsCmdCb, this, tcpNoDelay());
}

void DynamixelHandler::publishCurrentStates(const ros::Time& cur_time)
{
  dynamixel::GroupSyncRead pwm_sync_read(poh_, pah_, kAddrPresentPwm, 2);
  dynamixel::GroupSyncRead current_sync_read(poh_, pah_, kAddrPresentCurrent, 2);
  dynamixel::GroupSyncRead pos_sync_read(poh_, pah_, kAddrPresentPosition, 4);
  dynamixel::GroupSyncRead vel_sync_read(poh_, pah_, kAddrPresentVelocity, 4);
  dynamixel::GroupSyncRead voltage_sync_read(poh_, pah_, kAddrPresentInputVoltage, 2);
  dynamixel::GroupSyncRead temp_sync_read(poh_, pah_, kAddrPresentTemperature, 1);
  if (readSyncPacket(pwm_sync_read) < 0)
    rosError(name_, "Failed to receive a sync packet of present PWM.");
  if (readSyncPacket(current_sync_read) < 0)
    rosError(name_, "Failed to receive a sync packet of present current.");
  if (readSyncPacket(pos_sync_read) < 0)
    rosError(name_, "Failed to receive a sync packet of present position.");
  if (readSyncPacket(vel_sync_read) < 0)
    rosError(name_, "Failed to receive a sync packet of present velocity.");
  if (readSyncPacket(voltage_sync_read) < 0)
    rosError(name_, "Failed to receive a sync packet of present input voltage.");
  if (readSyncPacket(temp_sync_read) < 0)
    rosError(name_, "Failed to receive a sync packet of present temperature.");

  const auto motor_states = boost::make_shared<dynamixel_msgs::MotorStates>();
  const auto cur_js = boost::make_shared<sensor_msgs::JointState>();
  motor_states->header.stamp = cur_time;
  cur_js->header.stamp = cur_time;

  for (const auto& [name, cfg] : motors_)
  {
    const int16_t pwm_raw = pos_sync_read.getData(cfg.id, kAddrPresentPwm, 2);
    motor_state_.pwm = static_cast<double>(pwm_raw) * 100 / 855;

    const int16_t current_raw = current_sync_read.getData(cfg.id, kAddrPresentCurrent, 2);
    if (cfg.current_available)
    {
      motor_state_.current = static_cast<double>(current_raw) * cfg.current_scaling_factor / 1000;
      motor_state_.load = nan(kUnavailable);
    }
    else
    {
      motor_state_.current = nan(kUnavailable);
      motor_state_.load = static_cast<double>(current_raw) * 0.1;
    }

    const int32_t pos_raw = pos_sync_read.getData(cfg.id, kAddrPresentPosition, 4);
    motor_state_.position = dh_std::remap<double>(pos_raw, 0, 1 << 12, -M_PI, M_PI);

    const int32_t vel_raw = vel_sync_read.getData(cfg.id, kAddrPresentVelocity, 4);
    motor_state_.velocity = static_cast<double>(vel_raw) * kVelocityDecodeFactor;

    const uint16_t voltage_yaw = voltage_sync_read.getData(cfg.id, kAddrPresentInputVoltage, 2);
    motor_state_.input_voltage = static_cast<double>(voltage_yaw) * 0.1;

    const uint8_t temp_raw = temp_sync_read.getData(cfg.id, kAddrPresentTemperature, 1);
    motor_state_.temperature = static_cast<double>(temp_raw) * 1;

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
    const auto& tar_pos = positions->data[i];

    if (!motors_.contains(jnt_name))
    {
      rosError(name_, "Controller for joint '" << jnt_name << "' is not found.");
      continue;
    }

    const auto& cfg = motors_.at(jnt_name);

    if (cfg.operating_mode == kPositionControlMode)
      goal_positions[i] = dh_std::remap<double>(tar_pos, -M_PI, M_PI, 0, 1 << 12);
    else if (
      cfg.operating_mode == kExtendedPositionControlMode
      || cfg.operating_mode == kCurrentBasePositionControlMode)
      goal_positions[i] = tar_pos * (1 << 12) / (2 * M_PI);
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
    const auto& tar_vel = velocities->data[i];

    if (!motors_.contains(jnt_name))
    {
      rosError(name_, "Controller for joint '" << jnt_name << "' is not found.");
      continue;
    }

    const auto& cfg = motors_.at(jnt_name);

    if (cfg.operating_mode != kVelocityControlMode)
    {
      rosError(name_, "The operating mode of joint '" << jnt_name << "' is not velocity.");
      continue;
    }

    goal_velocities[i] = tar_vel / kVelocityDecodeFactor;
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

    if (!motors_.contains(jnt_name))
    {
      rosError(name_, "Controller for joint '" << jnt_name << "' is not found.");
      continue;
    }

    const auto& cfg = motors_.at(jnt_name);

    if (cfg.operating_mode != kCurrentControlMode && cfg.operating_mode != kPwmControlMode)
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
}  // namespace dynamixel_handler
