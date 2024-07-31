#include <rclcpp/rclcpp.hpp>
#include <ros/timer.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <std_srvs/SetBool.h>

#include <tobas_std_tools/range.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_msgs/Event.h>
#include <tobas_msgs/JointCommandArray.h>
#include <tobas_dynamixel_msgs/MotorStateArray.h>

namespace tobas_dynamixel_handler
{
struct DynamixelConfig
{
  uint8_t id;
  uint8_t operating_mode;

  bool current_available;
  double current_scaling_factor;  // code -> A

  double temp_limit;                       // [degC]
  tobas_std::Range<double> voltage_limit;  // [V]
  double pwm_limit;                        // [%]
  double current_limit;                    // [A]
  double acc_limit;                        // [rad/s^2]
  double vel_limit;                        // [rad/s]
  tobas_std::Range<double> pos_limit;      // [rad]
};

class DynamixelHandler : public tobas::BaseNode
{
  using self = DynamixelHandler;
  using super = tobas::BaseNode;

public:
  explicit DynamixelHandler(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());
  ~DynamixelHandler();

private:
  dynamixel::PortHandler* poh_;
  dynamixel::PacketHandler* pah_;
  std::unique_ptr<dynamixel::GroupSyncRead> pos_sync_read_;
  std::unique_ptr<dynamixel::GroupSyncRead> vel_sync_read_;
  std::unique_ptr<dynamixel::GroupSyncRead> current_sync_read_;
  std::unique_ptr<dynamixel::GroupSyncRead> pwm_sync_read_;
  std::unique_ptr<dynamixel::GroupSyncRead> voltage_sync_read_;
  std::unique_ptr<dynamixel::GroupSyncRead> temp_sync_read_;
  std::unique_ptr<dynamixel::GroupSyncRead> hes_sync_read_;
  std::unique_ptr<dynamixel::GroupSyncWrite> pos_sync_write_;
  std::unique_ptr<dynamixel::GroupSyncWrite> vel_sync_write_;

  std::vector<int32_t> goal_positions_;
  std::vector<int32_t> goal_velocities_;
  tobas_dynamixel_msgs::MotorState motor_state_;
  bool is_enabled_ = true;

  // rosparams
  rclcpp::V_string jnt_names_;
  std::string device_name_;
  float protocol_version_;
  size_t baudrate_;
  uint8_t return_delay_time_;
  bool read_pwm_;
  bool read_current_;
  bool read_velocity_;
  bool read_position_;
  bool read_voltage_;
  bool read_temperature_;
  std::unordered_map<std::string, DynamixelConfig> motors_;

  // PubSub
  rclcpp::Publisher motor_states_pub_;
  rclcpp::Subscriber event_sub_;
  rclcpp::Subscriber positions_sub_;
  rclcpp::Subscriber velocities_sub_;
  rclcpp::Subscriber efforts_sub_;

  // Service
  rclcpp::ServiceServer enable_torques_ss_;

  // Timer
  rclcpp::Timer main_timer_;

  void getRosParams();
  void registerPublishers();
  void registerSubscribers();
  bool setMinimumLatency();
  void getMotorConfigs();
  void publishCurrentStates(const rclcpp::Time& cur_time);
  bool enableTorques();
  bool disableTorques();
  void printHardwareErrorStatus();

  void eventCb(const tobas_msgs::EventConstPtr& event);
  void jointPositionsCmdCb(const tobas_msgs::JointCommandArrayConstPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::JointCommandArrayConstPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::JointCommandArrayConstPtr& efforts);

  bool enableTorquesServiceCb(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);
  void mainTimerCb(const rclcpp::TimerEvent& event);
};
}  // namespace tobas_dynamixel_handler
