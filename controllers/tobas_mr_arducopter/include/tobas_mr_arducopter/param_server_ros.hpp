#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>
#include <mavros_msgs/ParamSet.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/PoseTwist.h>

#include <tobas_mr_arducopter/ControllerConfig.h>

#include "./socket.hpp"

namespace tobas_mr_arducopter
{
/**
 * @brief ArduCopter Controller \n
 * cf. [ArduPilot Gazebo Plugin](https://github.com/ArduPilot/ardupilot_gazebo)
 */
class ParamServerRos : public tobas::BaseNode
{
  const std::string kParamSetSrvName = "mavros/param/set";

  using self = ParamServerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_mr_arducopter::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit ParamServerRos(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  // Dynamic Reconfigure Server
  ConfigServer server_;

  std::string param_id_;
  std::unordered_map<std::string, double> params_;
  mavros_msgs::ParamSet param_set_msg_;
  ros::ServiceClient param_set_sc_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;

  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace tobas_mr_arducopter
