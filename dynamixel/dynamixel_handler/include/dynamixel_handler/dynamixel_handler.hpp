#include <ros/ros.h>
#include <ros/timer.h>
#include <sensor_msgs/JointState.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/JointPositions.h>
#include <tobas_msgs/JointVelocities.h>
#include <tobas_msgs/JointEfforts.h>
#include <dynamixel_msgs/MotorStates.h>

namespace dynamixel_handler
{
struct DynamixelConfig
{
  uint8_t id;
  uint8_t operating_mode;
  bool current_available;
  double current_scaling_factor;
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

  dynamixel_msgs::MotorState motor_state_;

  // Buffers
  int16_t present_pwm_;
  int16_t present_current_;
  int32_t present_position_;
  int32_t present_velocity_;
  uint16_t present_input_voltage_;
  uint8_t present_temperature_;
  int32_t goal_position_;

  // rosparams
  ros::V_string jnt_names_;
  std::string device_name_;
  float protocol_version_;
  size_t baudrate_;
  size_t update_rate_;
  std::unordered_map<std::string, DynamixelConfig> motors_;

  // PubSub
  ros::Publisher motor_states_pub_;
  ros::Publisher cur_js_pub_;
  ros::Subscriber positions_sub_;
  ros::Subscriber velocities_sub_;
  ros::Subscriber efforts_sub_;

  // Timer
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void setMinimumLatency();
  void getMotorConfigs();
  void publishCurrentStates(const ros::Time& cur_time);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void jointPositionsCmdCb(const tobas_msgs::JointPositionsConstPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::JointVelocitiesConstPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::JointEffortsConstPtr& efforts);

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace dynamixel_handler
