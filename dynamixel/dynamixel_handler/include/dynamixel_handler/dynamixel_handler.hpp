#include <ros/ros.h>
#include <ros/timer.h>
#include <sensor_msgs/JointState.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

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

class DynamixelHandler
{
  using self = DynamixelHandler;

public:
  explicit DynamixelHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name);
  ~DynamixelHandler();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  const std::string name_;

  dynamixel::PortHandler* poh_;
  dynamixel::PacketHandler* pah_;

  dynamixel_msgs::MotorState motor_state_;
  sensor_msgs::JointStateConstPtr tar_js_;

  // Buffers
  int16_t present_pwm_;
  int16_t present_current_;
  int32_t present_position_;
  int32_t present_velocity_;
  uint16_t present_input_voltage_;
  uint8_t present_temperature_;

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
  ros::Subscriber tar_js_sub_;

  // Timer
  ros::Timer main_timer_;

  void getRosParams();
  void getMotorConfigs();
  void registerPubSub();
  void publishCurrentStates(const ros::Time& cur_time);
  void sendCommands();

  void targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js);
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace dynamixel_handler
