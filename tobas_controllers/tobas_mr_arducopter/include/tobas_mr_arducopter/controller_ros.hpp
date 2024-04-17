#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Odometry.h>

#include "./socket.hpp"

namespace tobas_mr_arducopter
{
/**
 * @brief ArduCopter Controller \n
 * cf. [ArduPilot Gazebo Plugin](https://github.com/ArduPilot/ardupilot_gazebo)
 */
class ControllerRos : public tobas::BaseNode
{
  using self = ControllerRos;
  using super = tobas::BaseNode;

public:
  explicit ControllerRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  const KDL::Rotation R_nwu_ned_ = KDL::Rotation::RotX(M_PI);

  bool ardupilot_online_ = false;
  size_t connection_timeout_count_ = 0;
  ArduPilotSocket socket_in_;
  ArduPilotSocket socket_out_;

  // rosparam
  std::vector<int> channels_;  // ArduPilotにおける各モータのチャンネル

  // Publishers
  ros::Publisher throttles_pub_;

  // Subscribers
  ros::Subscriber odom_sub_;

  void getRosParams();
  void receiveAndPublishMotorCommand(const ros::Time& imu_time);
  void sendState(const tobas_msgs::Odometry& odom);

  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
};
}  // namespace tobas_mr_arducopter
