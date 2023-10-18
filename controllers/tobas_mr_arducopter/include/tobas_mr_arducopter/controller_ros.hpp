#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_msgs/RotorSpeeds.h>

#include <tobas_mr_arducopter/ControllerConfig.h>

#include "./attitude_controller.hpp"
#include "./position_controller.hpp"

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
  // Drone
  tobas::Drone drone_;

  // Mutable variables
  tobas_msgs::PoseTwistConstPtr pt_;
  tobas_msgs::BatteryConstPtr battery_;
  tobas_msgs::PosVelAccYawPtr tar_pvay_;        // PosVelYawの目標値 (世界座標系)
  tobas_msgs::RollPitchYawThrustPtr tar_rpyt_;  // RollPitchYawThrustの目標値
  bool is_initialized_ = false;
  uint8_t cmd_level_ = tobas_msgs::CommandLevel::NORMAL;
  ros::Time t_last_loop_;

  // ArduPilot
  AttitudeController attitude_control_;
  PositionController pos_control_;

  // Publishers
  ros::Publisher rotor_speeds_pub_;
  ros::Publisher feedback_pub_;

  // Subscribers
  ros::Subscriber pt_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber pvay_sub_;
  ros::Subscriber rpyt_sub_;

  // Timers
  ros::Timer check_topics_timer_;

  // Dynamic Reconfigure Server
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady() const;
  bool isCommandLevelOk(const tobas_msgs::CommandLevel& level);

  /* ===== ArduCopter fast tasks ===== */
  /* Update rate controllers and output to roll, pitch and yaw actuators. */
  void runRateController();
  void motorsOutput();
  void readAHRS();
  void readInertia();
  void ckeckEkfReset();
  void updateFlightMode();
  void updateHomeFromEkf();
  void updateLandAndCrashDetectors();
  void updateRangeFinderTerrainOffset();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void posVelAccYawCb(const tobas_msgs::PosVelAccYawConstPtr& pvay);
  void rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpyt);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace tobas_mr_arducopter
