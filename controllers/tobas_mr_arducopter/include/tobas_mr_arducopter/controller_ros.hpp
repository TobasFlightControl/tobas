#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/PoseTwist.h>

#include <tobas_mr_arducopter/ControllerConfig.h>

#include "./socket.hpp"

namespace tobas_mr_arducopter
{
class ControllerRos : public tobas::BaseNode
{
  using self = ControllerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_mr_arducopter::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit ControllerRos(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  const KDL::Rotation R_nwu_ned_ = KDL::Rotation::RotX(M_PI);

  bool ardupilot_online_ = false;
  uint32_t connection_timeout_count_ = 0;
  ArduPilotSocket socket_in_;
  ArduPilotSocket socket_out_;

  // rosparam
  uint32_t max_connection_timeout_count_;
  uint32_t num_rotors_;
  std::vector<int> channels_;  // ArduPilotにおける各モータのチャンネル

  // Publishers
  ros::Publisher throttles_pub_;

  // Subscribers
  ros::Subscriber pt_sub_;

  // Dynamic Reconfigure Server
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void initializeSockets();
  void receiveAndPublishMotorCommand(const ros::Time& imu_time);
  void sendState(const tobas_msgs::PoseTwist& pt);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt);

  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace tobas_mr_arducopter
