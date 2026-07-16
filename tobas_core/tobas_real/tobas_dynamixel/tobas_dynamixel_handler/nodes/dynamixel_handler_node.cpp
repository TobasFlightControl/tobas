// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <filesystem>
#include <fstream>

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <tobas_dynamixel_ros_interface/ros_interface.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_std_tools/range.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <tobas_dynamixel_msgs/msg/motor_command_array.hpp>
#include <tobas_dynamixel_msgs/msg/motor_state_array.hpp>

#include "./constants.hpp"

using namespace std::chrono_literals;
namespace ch = std::chrono;
namespace fs = std::filesystem;

namespace tobas
{
namespace dxl
{
struct DynamixelConfig
{
  uint8_t id;
  uint8_t operating_mode;

  bool current_available;
  double current_scaling_factor;  // code -> A

  double temp_limit;                // [degC]
  st::Range<double> voltage_limit;  // [V]
  double pwm_limit;                 // [%]
  double current_limit;             // [A]
  double acc_limit;                 // [rad/s^2]
  double vel_limit;                 // [rad/s]
  st::Range<double> pos_limit;      // [rad]
};

class DynamixelHandlerNode : public BaseNode
{
  using self = DynamixelHandlerNode;
  using super = BaseNode;

public:
  explicit DynamixelHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DynamixelHandlerNode();

private:
  dynamixel::PortHandler* poh_;
  dynamixel::PacketHandler* pah_;
  std::unique_ptr<dynamixel::GroupSyncRead> core_sync_read_;
  std::unique_ptr<dynamixel::GroupSyncRead> hes_sync_read_;
  std::unique_ptr<dynamixel::GroupSyncWrite> pos_sync_write_;
  std::unique_ptr<dynamixel::GroupSyncWrite> vel_sync_write_;

  std::vector<int32_t> goal_positions_;
  std::vector<int32_t> goal_velocities_;
  tobas_dynamixel_msgs::msg::MotorState motor_state_;
  bool is_enabled_ = true;

  // Parameters
  std::vector<std::string> jnt_names_;
  std::string device_name_;
  float protocol_version_;
  int baudrate_;
  uint8_t return_delay_time_;
  int state_pub_rate_;
  std::unordered_map<std::string, DynamixelConfig> motors_;

  // PubSub
  ros2::PublisherPtr<tobas_dynamixel_msgs::msg::MotorStateArray> motor_states_pub_;
  ros2::SubscriberPtr<tobas_dynamixel_msgs::msg::MotorCommandArray> positions_sub_;
  ros2::SubscriberPtr<tobas_dynamixel_msgs::msg::MotorCommandArray> velocities_sub_;
  ros2::SubscriberPtr<tobas_dynamixel_msgs::msg::MotorCommandArray> efforts_sub_;

  // Service
  ros2::ServiceServerPtr<std_srvs::srv::SetBool> enable_torques_ss_;

  // Timer
  ros2::TimerPtr pub_states_timer_;

  bool getStaticRosParams();
  void registerPublishers();
  void registerSubscribers();
  bool setMinimumLatency();
  bool getMotorConfigs();
  bool enableTorques();
  bool disableTorques();
  void printHardwareErrorStatus();

  void positionsCmdCb(const tobas_dynamixel_msgs::msg::MotorCommandArray::ConstSharedPtr& positions);
  void velocitiesCmdCb(const tobas_dynamixel_msgs::msg::MotorCommandArray::ConstSharedPtr& velocities);
  void effortsCmdCb(const tobas_dynamixel_msgs::msg::MotorCommandArray::ConstSharedPtr& efforts);

  void enableTorquesCb(
    const std_srvs::srv::SetBool::Request::ConstSharedPtr& req,
    const std_srvs::srv::SetBool::Response::SharedPtr& res);

  void publishCurrentStatesTimerCb();
};

DynamixelHandlerNode::DynamixelHandlerNode(const rclcpp::NodeOptions& options)
  : super("dynamixel_handler", nodeOptions_Default(options))
{
  // Get ROS parameters
  if (!getStaticRosParams()) {
    return;
  }

  // Get the device absolute path
  std::error_code ec;
  device_name_ = fs::canonical(device_name_, ec);
  if (ec) {
    TOBAS_ERROR(ec.message());
    return;
  }

  // Initialize Dynamixel SDK
  poh_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
  pah_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  // Current + Velocity + Position
  constexpr auto kCoreReadBegin = address::kPresentCurrent;
  constexpr auto kCoreReadEnd = address::kVelocityTrajectory;
  constexpr auto kCoreReadLength = kCoreReadEnd - kCoreReadBegin;
  core_sync_read_ = std::make_unique<dynamixel::GroupSyncRead>(poh_, pah_, kCoreReadBegin, kCoreReadLength);

  hes_sync_read_ = std::make_unique<dynamixel::GroupSyncRead>(poh_, pah_, address::kHardwareErrorStatus, 1);

  pos_sync_write_ = std::make_unique<dynamixel::GroupSyncWrite>(poh_, pah_, address::kGoalPosition, 4);
  vel_sync_write_ = std::make_unique<dynamixel::GroupSyncWrite>(poh_, pah_, address::kGoalVelocity, 4);

  // Open serial port
  if (!poh_->openPort()) {
    TOBAS_ERROR("Failed to open port \"", device_name_, "\"");
    return;
  }

  // Set baudrate
  if (!poh_->setBaudRate(baudrate_)) {
    TOBAS_ERROR("Failed to set baudrate to ", baudrate_);
    return;
  }

  // Get motor configurations
  if (!getMotorConfigs()) {
    return;
  }

  for (const auto& [name, cfg] : motors_) {
    // Add parameters to dynamixel::GroupSyncRead objects
    if (!core_sync_read_->addParam(cfg.id) || !hes_sync_read_->addParam(cfg.id)) {
      TOBAS_ERROR("Motor ID ", static_cast<int>(cfg.id), " is duplicated.");
      return;
    }

    // Disable torque
    if (pah_->write1ByteTxRx(poh_, cfg.id, address::kToruqeEnable, torque_enable::kDisable) < 0) {
      TOBAS_ERROR("Failed to disable torque of \"", name, "\".");
      return;
    }

    // Set return delay time
    if (pah_->write1ByteTxRx(poh_, cfg.id, address::kReturnDelayTime, return_delay_time_) < 0) {
      TOBAS_ERROR("Failed to set return delay time of \"", name, "\".");
      return;
    }

    // Set operating mode
    if (pah_->write1ByteTxRx(poh_, cfg.id, address::kOperatingMode, cfg.operating_mode) < 0) {
      TOBAS_ERROR("Failed to set operating mode of \"", name, "\".");
      return;
    }

    // Enable torque
    if (pah_->write1ByteTxRx(poh_, cfg.id, address::kToruqeEnable, torque_enable::kEnable) < 0) {
      TOBAS_ERROR("Failed to enable torque of \"", name, "\".");
      return;
    }

    TOBAS_INFO("\"", name, "\" has been initialized.");
  }

  // Reduce latency
  if (!setMinimumLatency()) {
    TOBAS_ERROR("Failed to set communication latency.");
    return;
  }

  // Register publishers and subscribers
  registerPublishers();
  registerSubscribers();

  // Register service servers
  enable_torques_ss_ = createService<std_srvs::srv::SetBool>(service::kEnableTorques, &self::enableTorquesCb, this);

  // Start main timer with maximum rate
  const auto state_pub_period = state_pub_rate_ > 0 ? ch::duration<double>(1.0 / state_pub_rate_) : 0ms;
  pub_states_timer_ = createWallTimer(state_pub_period, &self::publishCurrentStatesTimerCb, this);
}

DynamixelHandlerNode::~DynamixelHandlerNode()
{
  // Disable torque
  disableTorques();

  // Close serial port
  poh_->closePort();
}

bool DynamixelHandlerNode::getStaticRosParams()
{
  jnt_names_ = getStringArrayParam("joint_names", {});
  if (jnt_names_.empty()) {
    TOBAS_ERROR("Joint names are not specified.");
    return false;
  }

  device_name_ = getStringParam("device_name", "/dev/ttyUSB0");
  protocol_version_ = getDoubleParam("protocol_version", 2.0);
  baudrate_ = getIntParam("baudrate", 57600);

  const auto return_delay_time = getIntParam("return_delay_time", 0);
  if (return_delay_time < 0 || UINT8_MAX < return_delay_time) {
    TOBAS_ERROR("Invalid return delay time: ", return_delay_time);
    return false;
  }
  return_delay_time_ = static_cast<uint8_t>(return_delay_time);

  state_pub_rate_ = getIntParam("state_publish_rate", 0);
  if (state_pub_rate_ < 0) {
    TOBAS_ERROR("State publish rate must be non-negative.");
    return false;
  }

  return true;
}

void DynamixelHandlerNode::registerPublishers()
{
  motor_states_pub_ = createPublisher<tobas_dynamixel_msgs::msg::MotorStateArray>(topic::kMotorStates);
}

void DynamixelHandlerNode::registerSubscribers()
{
  positions_sub_ = createSubscriber(topic::kPositionCommand, &self::positionsCmdCb, this);
  velocities_sub_ = createSubscriber(topic::kVelocityCommand, &self::velocitiesCmdCb, this);
  efforts_sub_ = createSubscriber(topic::kEffortCommand, &self::effortsCmdCb, this);
}

bool DynamixelHandlerNode::setMinimumLatency()
{
  const auto pos = device_name_.rfind('/');
  const auto port = pos == std::string::npos ? device_name_ : device_name_.substr(pos + 1);
  const auto file_path = "/sys/bus/usb-serial/devices/" + port + "/latency_timer";

  std::ofstream file(file_path);
  if (!file.is_open()) {
    return false;
  }

  file << 1;  // `uint8_t` did not take effect.
  file.close();

  TOBAS_INFO("Communication latency is updated successfully.");
  return true;
}

bool DynamixelHandlerNode::getMotorConfigs()
{
  DynamixelConfig cfg;
  std::string operating_mode;
  std::unordered_set<uint8_t> used_ids;
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

  for (const auto& name : jnt_names_) {
    // ID
    const auto id = getIntParam(name + ".id", -1);
    if (id < 0) {
      TOBAS_ERROR("Please specify ID for \"", name, "\".");
      return false;
    }
    if (id > UINT8_MAX) {
      TOBAS_ERROR("Motor ID must be in range of (0, ", UINT8_MAX, ").");
      return false;
    }
    if (used_ids.contains(id)) {
      TOBAS_ERROR("Motor ID ", id, " is duplicated.");
      return false;
    }
    cfg.id = id;
    used_ids.insert(id);

    // Current scaling factor: https://www.besttechnology.co.jp/modules/knowledge/?X%20Series%20Control%20table#y2cc93fd
    if (pah_->read2ByteTxRx(poh_, cfg.id, address::kModelNumber, &model_number) < 0) {
      TOBAS_ERROR("Failed to get model number of \"", name, "\".");
      return false;
    }
    switch (model_number) {
      case model_number::kXC330M181:
      case model_number::kXC330T181:
      case model_number::kXC330T288:
        cfg.current_available = true;
        cfg.current_scaling_factor = 1.0;
        break;
      case model_number::kXL430W250:
      case model_number::k2XL430W250:
      case model_number::kXC430W150:
      case model_number::kXC430W240:
      case model_number::k2XC430W250:
        cfg.current_available = false;
        cfg.current_scaling_factor = 0.0;
        break;
      case model_number::kXM430W210:
      case model_number::kXH430W210:
      case model_number::kXM430W350:
      case model_number::kXH430W350:
      case model_number::kXM540W150:
      case model_number::kXH540W150:
      case model_number::kXH540V150:
      case model_number::kXM540W270:
      case model_number::kXH540W270:
      case model_number::kXH540V270:
      case model_number::kXW540T140:
      case model_number::kXW540T260:
        cfg.current_available = true;
        cfg.current_scaling_factor = 2.69;
        break;
      case model_number::kXH430V210:
      case model_number::kXH430V350:
        cfg.current_available = true;
        cfg.current_scaling_factor = 1.34;
        break;
      default:
        TOBAS_WARN("Current scaling factor for model number ", model_number, " is unknown.");
        cfg.current_available = false;
        cfg.current_scaling_factor = 0.0;
    }
    cfg.current_scaling_factor *= 1e-3;  // mA -> A

    // Operating mode
    operating_mode = getStringParam(name + ".operating_mode", "");
    if (operating_mode.empty()) {
      TOBAS_ERROR("Please specify the operating mode for \"", name, "\".");
      return false;
    }
    if (operating_mode == "current") {
      if (!cfg.current_available) {
        TOBAS_ERROR("Current control mode is unavailable for \"", name, "\".");
        return false;
      }
      cfg.operating_mode = operating_mode::kPosition;
    }
    else if (operating_mode == "velocity") {
      cfg.operating_mode = operating_mode::kVelocity;
    }
    else if (operating_mode == "position") {
      cfg.operating_mode = operating_mode::kPosition;
    }
    else if (operating_mode == "extended_position") {
      cfg.operating_mode = operating_mode::kExtendedPosition;
    }
    else if (operating_mode == "current_base_position") {
      if (!cfg.current_available) {
        TOBAS_ERROR("Current-base position control mode is unavailable for \"", name, "\".");
        return false;
      }
      cfg.operating_mode = operating_mode::kCurrentBasePosition;
    }
    else if (operating_mode == "pwm") {
      cfg.operating_mode = operating_mode::kPwm;
    }
    else {
      TOBAS_ERROR("Unknown operating mode for \"", name, "\".");
      return false;
    }

    // Limits
    if (pah_->read1ByteTxRx(poh_, cfg.id, address::kTemperatureLimit, &temp_limit) == 0) {
      cfg.temp_limit = static_cast<double>(temp_limit) * scale_factor::kTemperature;
    }
    else {
      TOBAS_ERROR("Failed to get temperature limit of \"", name, "\".");
      return false;
    }

    if (pah_->read2ByteTxRx(poh_, cfg.id, address::kMaxVoltageLimit, &max_voltage_limit) == 0) {
      cfg.voltage_limit.upper = static_cast<double>(max_voltage_limit) * scale_factor::kVoltage;
    }
    else {
      TOBAS_ERROR("Failed to get maximum voltage limit of \"", name, "\".");
      return false;
    }

    if (pah_->read2ByteTxRx(poh_, cfg.id, address::kMinVoltageLimit, &min_voltage_limit) == 0) {
      cfg.voltage_limit.lower = static_cast<double>(min_voltage_limit) * scale_factor::kVoltage;
    }
    else {
      TOBAS_ERROR("Failed to get minimum voltage limit of \"", name, "\".");
      return false;
    }

    if (pah_->read2ByteTxRx(poh_, cfg.id, address::kPwmLimit, &pwm_limit) == 0) {
      cfg.pwm_limit = static_cast<double>(pwm_limit) * scale_factor::kPwm;
    }
    else {
      TOBAS_ERROR("Failed to get PWM limit of \"", name, "\".");
      return false;
    }

    if (cfg.current_available) {
      if (pah_->read2ByteTxRx(poh_, cfg.id, address::kCurrentLimit, &current_limit) == 0) {
        cfg.current_limit = static_cast<double>(current_limit) * cfg.current_scaling_factor;
      }
      else {
        TOBAS_ERROR("Failed to get current limit of \"", name, "\".");
        return false;
      }
    }
    else {
      cfg.current_limit = NAN;
    }

    if (pah_->read4ByteTxRx(poh_, cfg.id, address::kAccelerationLimit, &acc_limit) == 0) {
      cfg.acc_limit = static_cast<double>(acc_limit) * scale_factor::kAcceleration;
    }
    else {
      TOBAS_ERROR("Failed to get acceleration limit of \"", name, "\".");
      return false;
    }

    if (pah_->read4ByteTxRx(poh_, cfg.id, address::kVelocityLimit, &vel_limit) == 0) {
      cfg.vel_limit = static_cast<double>(vel_limit) * scale_factor::kVelocity;
    }
    else {
      TOBAS_ERROR("Failed to get velocity limit of \"", name, "\".");
      return false;
    }

    if (pah_->read4ByteTxRx(poh_, cfg.id, address::kMaxPositionLimit, &max_pos_limit) == 0) {
      cfg.pos_limit.upper = math::remap<double>(max_pos_limit, 0, 1 << 12, -M_PI, M_PI);
    }
    else {
      TOBAS_ERROR("Failed to get maximum position limit of \"", name, "\".");
      return false;
    }

    if (pah_->read4ByteTxRx(poh_, cfg.id, address::kMinPositionLimit, &min_pos_limit) == 0) {
      cfg.pos_limit.lower = math::remap<double>(min_pos_limit, 0, 1 << 12, -M_PI, M_PI);
    }
    else {
      TOBAS_ERROR("Failed to get minimum position limit of \"", name, "\".");
      return false;
    }

    // Insert motor config
    motors_[name] = cfg;
  }

  return true;
}

bool DynamixelHandlerNode::enableTorques()
{
  for (const auto& [name, cfg] : motors_) {
    if (pah_->write1ByteTxRx(poh_, cfg.id, address::kToruqeEnable, torque_enable::kEnable) < 0) {
      TOBAS_ERROR("Failed to enable torque of \"", name, "\".");
      return false;
    }
  }

  is_enabled_ = true;
  return true;
}

bool DynamixelHandlerNode::disableTorques()
{
  for (const auto& [name, cfg] : motors_) {
    if (pah_->write1ByteTxRx(poh_, cfg.id, address::kToruqeEnable, torque_enable::kDisable) < 0) {
      TOBAS_ERROR("Failed to disable torque of \"", name, "\".");
      return false;
    }
  }

  is_enabled_ = false;
  return true;
}

void DynamixelHandlerNode::printHardwareErrorStatus()
{
  if (hes_sync_read_->txRxPacket() < 0) {
    TOBAS_ERROR("Failed to receive a sync packet of hardware error status.");
    return;
  }

  for (const auto& [name, cfg] : motors_) {
    const uint8_t hes = hes_sync_read_->getData(cfg.id, address::kHardwareErrorStatus, 1);
    if (hes & hardware_error::kInputVoltage) {
      TOBAS_ERROR("Input voltage error in \"", name, "\"");
    }
    if (hes & hardware_error::kHallSensor) {
      TOBAS_ERROR("Hall sensor error in \"", name, "\"");
    }
    if (hes & hardware_error::kOverheating) {
      TOBAS_ERROR("Overheating error in \"", name, "\"");
    }
    if (hes & hardware_error::kMotorEncoder) {
      TOBAS_ERROR("Motor encoder error in \"", name, "\"");
    }
    if (hes & hardware_error::kElectricalShock) {
      TOBAS_ERROR("Electrical shock error in \"", name, "\"");
    }
    if (hes & hardware_error::kOverload) {
      TOBAS_ERROR("Overload error in \"", name, "\"");
    }
  }
}

void DynamixelHandlerNode::positionsCmdCb(const tobas_dynamixel_msgs::msg::MotorCommandArray::ConstSharedPtr& positions)
{
  if (!is_enabled_) {
    TOBAS_ERROR("Motors are disabled. You cannot command positions.");
    return;
  }

  const auto size = positions->commands.size();
  goal_positions_.resize(size);
  pos_sync_write_->clearParam();

  for (size_t i = 0; i < size; ++i) {
    const auto& jnt_name = positions->commands[i].name;
    if (!motors_.contains(jnt_name)) {
      TOBAS_ERROR("Controller for joint \"", jnt_name, "\" is not found.");
      continue;
    }

    const auto& cfg = motors_.at(jnt_name);
    auto tar_pos = positions->commands[i].data;
    if (cfg.operating_mode == operating_mode::kPosition) {
      if (cfg.pos_limit.clamp(tar_pos, tar_pos)) {
        TOBAS_WARN("Target position of joint \"", jnt_name, "\" is out of limit. The value is clamped to ", tar_pos);
      }
      goal_positions_[i] = math::remap<double>(tar_pos, -M_PI, M_PI, 0, 1 << 12);
    }
    else if (
      cfg.operating_mode == operating_mode::kExtendedPosition ||
      cfg.operating_mode == operating_mode::kCurrentBasePosition) {
      goal_positions_[i] = tar_pos / scale_factor::kPosition;
    }
    else {
      TOBAS_ERROR("The operating mode of joint \"", jnt_name, "\" is not position.");
      continue;
    }

    if (!pos_sync_write_->addParam(cfg.id, (uint8_t*)&goal_positions_[i])) {
      TOBAS_ERROR("Failed to set goal position of joint \"", jnt_name, "\".");
    }
  }

  if (pos_sync_write_->txPacket() < 0) {
    TOBAS_ERROR("Failed to transmit a sync write instruction packet of positions.");
  }
}

void DynamixelHandlerNode::velocitiesCmdCb(const tobas_dynamixel_msgs::msg::MotorCommandArray::ConstSharedPtr& velocities)
{
  if (!is_enabled_) {
    TOBAS_ERROR("Motors are disabled. You cannot command velocities.");
    return;
  }

  const auto size = velocities->commands.size();
  goal_velocities_.resize(size);
  vel_sync_write_->clearParam();

  for (size_t i = 0; i < size; ++i) {
    const auto& jnt_name = velocities->commands[i].name;
    if (!motors_.contains(jnt_name)) {
      TOBAS_ERROR("Controller for joint \"", jnt_name, "\" is not found.");
      continue;
    }

    const auto& cfg = motors_.at(jnt_name);
    if (cfg.operating_mode != operating_mode::kVelocity) {
      TOBAS_ERROR("The operating mode of joint \"", jnt_name, "\" is not velocity.");
      continue;
    }

    auto tar_vel = velocities->commands[i].data;
    if (std::abs(tar_vel) > cfg.vel_limit) {
      tar_vel = std::clamp(tar_vel, -cfg.vel_limit, cfg.vel_limit);
      TOBAS_WARN("Target velocity of joint \"", jnt_name, "\" is out of limit. The value is clamped to ", tar_vel);
    }

    goal_velocities_[i] = tar_vel / scale_factor::kVelocity;
    if (!vel_sync_write_->addParam(cfg.id, (uint8_t*)&goal_velocities_[i])) {
      TOBAS_ERROR("Failed to set goal velocity of joint \"", jnt_name, "\".");
      continue;
    }
  }

  if (vel_sync_write_->txPacket() < 0) {
    TOBAS_ERROR("Failed to transmit a sync write instruction packet of velocities.");
  }
}

void DynamixelHandlerNode::effortsCmdCb(const tobas_dynamixel_msgs::msg::MotorCommandArray::ConstSharedPtr& efforts)
{
  if (!is_enabled_) {
    TOBAS_ERROR("Motors are disabled. You cannot command efforts.");
    return;
  }

  const auto size = efforts->commands.size();

  for (size_t i = 0; i < size; ++i) {
    const auto& jnt_name = efforts->commands[i].name;
    if (!motors_.contains(jnt_name)) {
      TOBAS_ERROR("Controller for joint \"", jnt_name, "\" is not found.");
      continue;
    }

    const auto& cfg = motors_.at(jnt_name);
    if (cfg.operating_mode != operating_mode::kCurrent && cfg.operating_mode != operating_mode::kPwm) {
      TOBAS_ERROR("The operating mode of joint \"", jnt_name, "\" is not effort.");
      continue;
    }

    TOBAS_WARN("Effort control is not implemented yet.");  // TODO
  }
}

void DynamixelHandlerNode::enableTorquesCb(
  const std_srvs::srv::SetBool::Request::ConstSharedPtr& req,
  const std_srvs::srv::SetBool::Response::SharedPtr& res)
{
  if (req->data) {
    if (enableTorques()) {
      res->success = true;
      res->message = "Torques enabled.";
    }
    else {
      res->success = false;
      res->message = "Failed to enable torques.";
    }
  }
  else {
    if (disableTorques()) {
      res->success = true;
      res->message = "Torques disabled.";
    }
    else {
      res->success = false;
      res->message = "Failed to disable torques.";
    }
  }
}

void DynamixelHandlerNode::publishCurrentStatesTimerCb()
{
  // Create motor states message
  auto motor_states = std::make_unique<tobas_dynamixel_msgs::msg::MotorStateArray>();
  motor_states->header.stamp = now();

  // Read packets
  if (core_sync_read_->txRxPacket() < 0) {
    TOBAS_ERROR("Failed to receive a sync packet of present states. Disabling torques.");
    disableTorques();
    printHardwareErrorStatus();  // FIXME: HES cannot be read after motor shutdown.
    return;
  }

  // Compute joint states in the SI unit system
  for (const auto& [name, cfg] : motors_) {
    motor_state_.name = name;

    const int32_t pos_raw = core_sync_read_->getData(cfg.id, address::kPresentPosition, 4);
    motor_state_.position = math::remap<double>(pos_raw, 0, 1 << 12, -M_PI, M_PI);

    const int32_t vel_raw = core_sync_read_->getData(cfg.id, address::kPresentVelocity, 4);
    motor_state_.velocity = static_cast<double>(vel_raw) * scale_factor::kVelocity;

    const int16_t current_raw = core_sync_read_->getData(cfg.id, address::kPresentCurrent, 2);
    if (cfg.current_available) {
      motor_state_.current = static_cast<double>(current_raw) * cfg.current_scaling_factor;
      motor_state_.load = NAN;
    }
    else {
      motor_state_.current = NAN;
      motor_state_.load = static_cast<double>(current_raw) * scale_factor::kLoad;
    }

    motor_states->states.push_back(motor_state_);
  }

  // Publish motor states message
  motor_states_pub_->publish(std::move(motor_states));
}
}  // namespace dxl
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::dxl::DynamixelHandlerNode)
