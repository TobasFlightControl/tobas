#include <ros/ros.h>
#include <ros/timer.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <tobas_std_tools/range.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/JointPositions.h>
#include <tobas_msgs/JointVelocities.h>
#include <tobas_msgs/JointEfforts.h>
#include <tobas_dynamixel_msgs/MotorStates.h>

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
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());
  ~DynamixelHandler();

private:
  dynamixel::PortHandler* poh_;
  dynamixel::PacketHandler* pah_;

  tobas_dynamixel_msgs::MotorState motor_state_;
  bool is_enabled_ = true;

  // rosparams
  ros::V_string jnt_names_;
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
  ros::Publisher motor_states_pub_;
  ros::Subscriber positions_sub_;
  ros::Subscriber velocities_sub_;
  ros::Subscriber efforts_sub_;

  // Timer
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool setMinimumLatency();
  void getMotorConfigs();
  void publishCurrentStates(const ros::Time& cur_time);
  void enableTorques();
  void disableTorques();
  void printHardwareErrorStatus();
  int readSyncPacket(dynamixel::GroupSyncRead& sync_read);

  void jointPositionsCmdCb(const tobas_msgs::JointPositionsConstPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::JointVelocitiesConstPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::JointEffortsConstPtr& efforts);

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_dynamixel_handler
