#include <dh_std_tools/vector.hpp>
#include <dh_eigen_tools/geometry.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_mr_arducopter/ControllerFeedback.h>

#include "../include/tobas_mr_arducopter/controller_ros.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_mr_arducopter
{
ControllerRos::ControllerRos(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name), server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  registerPublishers();
  registerSubscribers();

  check_topics_timer_ =
    nh_.createTimer(ros::Duration(tobas::kCheckTopicsTimerPeriod), &self::checkTopicsTimerCb, this);

  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ControllerRos::getRosParams()
{
}

void ControllerRos::registerPublishers()
{
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>(tobas::kRotorCmdTopic, 1);
  feedback_pub_ =
    nh_.advertise<tobas_mr_arducopter::ControllerFeedback>(tobas::kControllerFeedbackTopic, 1);
}

void ControllerRos::registerSubscribers()
{
  event_sub_ = nh_.subscribe(tobas::kEventTopic, 1, &self::eventCb, this, tcpNoDelay());
  pt_sub_ = nh_.subscribe(tobas::kPoseTwistTopic, 1, &self::poseTwistCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryTopic, 1, &self::batteryCb, this, tcpNoDelay());

  pvay_sub_ =
    nh_.subscribe(tobas::kPosVelAccYawCmdTopic, 1, &self::posVelAccYawCb, this, tcpNoDelay());
  rpyt_sub_ = nh_.subscribe(tobas::kRpyThrustCmdTopic, 1, &self::rpyThrustCb, this, tcpNoDelay());
}

bool ControllerRos::isReady() const
{
  if (pt_ == nullptr)
    return false;

  if (battery_ == nullptr)
    return false;

  return true;
}

void ControllerRos::runRateController()
{
  const auto dt = (pt_->header.stamp - t_last_loop_).toSec();
  t_last_loop_ = pt_->header.stamp;

  attitude_control_.setDt(dt);
  pos_control_.setDt(dt);

  attitude_control_.rateControllerRun(pt_->twist.rot.data);
}

void ControllerRos::motorsOutput()
{
  // TODO
}

void ControllerRos::readAHRS()
{
  // TODO
}

void ControllerRos::readInertia()
{
  // TODO
}

void ControllerRos::ckeckEkfReset()
{
  // TODO
}

void ControllerRos::updateFlightMode()
{
  // TODO
}

void ControllerRos::updateHomeFromEkf()
{
  // TODO
}

void ControllerRos::updateLandAndCrashDetectors()
{
  // TODO
}

void ControllerRos::updateRangeFinderTerrainOffset()
{
  // TODO
}

bool ControllerRos::isCommandLevelOk(const tobas_msgs::CommandLevel& level)
{
  if (level.data < cmd_level_)
  {
    rosErrorThrottle(
      tobas::kCommandLevelErrorPeriod, name_,
      "The command is ignored because its level " << static_cast<int>(level.data)
                                                  << "is lower than the current command level "
                                                  << static_cast<int>(cmd_level_) << ".");
    return false;
  }

  if (level.data > cmd_level_)
  {
    rosInfo(
      name_, "The command level is raised from " << static_cast<int>(cmd_level_) << " to "
                                                 << static_cast<int>(level.data) << ".");
    cmd_level_ = level.data;
  }

  return true;
}

void ControllerRos::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void ControllerRos::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  pt_ = pt;

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      t_last_loop_ = pt->header.stamp;
      is_initialized_ = true;
      rosInfo(name_, "Controller is ready.");
    }
    return;
  }

  // Create a feedback message
  auto feedback = boost::make_shared<tobas_mr_arducopter::ControllerFeedback>();
  feedback->header.stamp = pt->header.stamp;

  runRateController();
  motorsOutput();
  readAHRS();
  readInertia();
  ckeckEkfReset();
  updateFlightMode();
  updateHomeFromEkf();
  updateLandAndCrashDetectors();
  updateRangeFinderTerrainOffset();
}

void ControllerRos::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void ControllerRos::posVelAccYawCb(const tobas_msgs::PosVelAccYawConstPtr& pvay)
{
  if (pt_ == nullptr)
    return;

  // コマンドレベルの処理
  if (!isCommandLevelOk(pvay->level))
    return;

  // コマンドを更新
  tar_pvay_ = boost::make_shared<tobas_msgs::PosVelAccYaw>(*pvay);

  // グローバル座標系に変換
  if (!tobas::changeFrame(tobas_msgs::FrameId::GLOBAL, pt_->pose.euler, *tar_pvay_))
  {
    rosError(name_, "Failed to change command frame. Probably the frame id is invalid.");
    tar_pvay_ = nullptr;
    return;
  }
}

void ControllerRos::rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpyt)
{
  if (pt_ == nullptr)
    return;

  if (!isCommandLevelOk(rpyt->level))
    return;

  // 外側の制御を止める
  tar_pvay_ = nullptr;

  // コマンドを更新
  tar_rpyt_ = boost::make_shared<tobas_msgs::RollPitchYawThrust>(*rpyt);
}

void ControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (battery_ == nullptr)
    rosWarn(name_, "Battery state is not received yet.");

  if (pt_ == nullptr)
    rosWarn(name_, "Pose & Twist is not received yet.");
}

void ControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  // TODO

  rosInfo(name_, "Dynamic parameters are updated.");
}
}  // namespace tobas_mr_arducopter
