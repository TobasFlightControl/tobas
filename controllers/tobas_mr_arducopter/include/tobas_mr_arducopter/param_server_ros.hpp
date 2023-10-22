#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <geometry_msgs/PoseStamped.h>
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
  bool is_first_local_pos_ = true;
  bool is_first_update_ = true;
  bool is_init_params_set_ = false;
  dynamic_reconfigure::ConfigConstPtr init_cfg_;
  std::unordered_map<std::string, int> int_params_;
  std::unordered_map<std::string, double> double_params_;

  mavros_msgs::ParamSet param_set_msg_;
  ros::ServiceClient param_set_sc_;

  ros::Subscriber local_pos_sub_;
  ros::Subscriber param_updates_sub_;

  ros::Timer set_init_params_timer_;
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void setParams(const dynamic_reconfigure::ConfigConstPtr& cfg);
  void setParamsMap(const dynamic_reconfigure::ConfigConstPtr& cfg);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void localPositionCb(const geometry_msgs::PoseStampedConstPtr&);
  void paramUpdatesCb(const dynamic_reconfigure::ConfigConstPtr& cfg);

  void setInitParamsTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_mr_arducopter
