#include <dh_std_tools/zip.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/conversions/kdl_msg.hpp>
#include <tobas_msgs/JointEfforts.h>

#include "../include/tobas_manipulation/effort_controller_ros.hpp"
#include "../include/tobas_manipulation/common.hpp"

using namespace std;
using namespace KDL;

namespace tobas_manipulation
{
EffortControllerRos::EffortControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    cur_js_conv_(drone_.tree()),
    tar_js_conv_(drone_.tree()),
    pid_(drone_.tree()),
    server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  cur_js_conv_.updateInternalDataStructures();
  tar_js_conv_.updateInternalDataStructures();
  pid_.updateInternalDataStructures();

  if (!cur_js_conv_.setJointNames(drone_.postureDefiningJointNames()))
    ROS_THROW_NAMED(name_, "Failed to set joint names to joint converter.");
  if (!tar_js_conv_.setJointNames(drone_.postureDefiningJointNames()))
    ROS_THROW_NAMED(name_, "Failed to set joint names to joint converter.");

  jntarraynull_ = JntArray::Zero(drone_.tree().getNrOfJoints());

  // Set initial target joint states
  sensor_msgs::JointState init_tar_js;
  for (const auto& joint : drone_.jointConfigs())
  {
    init_tar_js.name.push_back(joint.name);
    init_tar_js.position.push_back(joint.init_pos);
    init_tar_js.velocity.push_back(0.);
    init_tar_js.effort.push_back(0.);
  }
  tar_js_ = boost::make_shared<sensor_msgs::JointState>(init_tar_js);

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void EffortControllerRos::getRosParams()
{
}

void EffortControllerRos::registerPublishers()
{
  efforts_pub_ = nh_.advertise<tobas_msgs::JointEfforts>(tobas::kJointEffortsCmdTopic, 1);
}

void EffortControllerRos::registerSubscribers()
{
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  cur_js_sub_ =
    nh_.subscribe(tobas::kJointStatesTopic, 1, &self::currentJointStateCb, this, tcpNoDelay());
  tar_js_sub_ =
    nh_.subscribe(tobas::kJointStatesCmdTopic, 1, &self::targetJointStateCb, this, tcpNoDelay());
  tar_cs_sub_ =
    nh_.subscribe(tobas::kCartStatesCmdTopic, 1, &self::targetCartStateCb, this, tcpNoDelay());
}

int EffortControllerRos::jointSpaceControl(JntArray& efforts)
{
  if (tar_js_conv_.jointStateToJntArray(*tar_js_) < 0)
  {
    rosError(
      name_, "Failed to convert target JointState to Jntarray: " << tar_js_conv_.errorMessage());
    return -1;
  }

  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  const auto& cur_qd = cur_js_conv_.getVelocitiesKDL();
  const auto& tar_q = tar_js_conv_.getPositionsKDL();
  const auto& tar_qd = tar_js_conv_.getVelocitiesKDL();

  // PIDで関節トルクを計算
  if (pid_.CartToJnt(cur_q, cur_qd, tar_q, tar_qd, jntarraynull_) < 0)
  {
    rosError(name_, "Joint space PID failed: " << pid_.errorMessage());
    return -1;
  }
  efforts = tar_js_conv_.getEffortsKDL() + pid_.getEfforts();  // FF + FB

  return 0;
}

int EffortControllerRos::taskSpaceControl(JntArray& efforts)
{
  // デカルト座標系の目標値を更新
  Frame tar_pi, T_W_B;
  FrameMap tar_p;
  TwistMap tar_v;
  AccelMap a_ff;
  WrenchMap f_ext;
  for (const auto& [seg_name, frame_id, pose, twist, accel, wrench] : dh_std::zip(
         tar_cs_->name, tar_cs_->frame_id, tar_cs_->pose, tar_cs_->twist, tar_cs_->accel,
         tar_cs_->wrench))
  {
    tobas::poseTobasToKDL(pose, tar_pi);
    auto tar_vi = twist;
    auto ai_ff = accel;
    auto fi_ext = wrench;
    switch (frame_id.data)
    {
      case tobas_msgs::FrameId::GLOBAL:
      {
        if (odom_ == nullptr)
        {
          rosWarnThrottle(
            kOdomNotReceivedWarnPeriod, name_,
            "Since odometry has not been received yet, commands in the global frame is ignored.");
          return -1;
        }

        tobas::poseTobasToKDL(odom_->pose, T_W_B);
        tar_pi = T_W_B.inverse() * tar_pi;  // T_B_P = T_B_W * T_W_P
        tar_vi = T_W_B.M.inverse(tar_vi);
        ai_ff = T_W_B.M.inverse(ai_ff);
        fi_ext = T_W_B.M.inverse(fi_ext);

        break;
      }
      case tobas_msgs::FrameId::LOCAL:
      {
        break;
      }
      default:
      {
        rosError(name_, "Unknown frame ID: " << static_cast<int>(frame_id.data));
        return -1;
      }
    }
    tar_p[seg_name] = tar_pi;  // Base -> Segment tip
    tar_v[seg_name] = tar_vi;
    a_ff[seg_name] = ai_ff;
    f_ext[seg_name] = fi_ext;
  }

  // PIDで関節トルクを計算
  // TODO: 毎周期クラスを初期化するのは無駄なので効率化
  TreeTaskSpacePID pid(drone_.tree(), tar_cs_->name);
  if (!pid.setLinearStiffness(lin_kp_))
    rosError(name_, "Failed to set linear stiffness.");
  if (!pid.setAngularStiffness(ang_kp_))
    rosError(name_, "Failed to set angular stiffness.");
  if (!pid.setLinearDamping(lin_kd_))
    rosError(name_, "Failed to set linear damping.");
  if (!pid.setAngularDamping(ang_kd_))
    rosError(name_, "Failed to set angular damping.");

  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  const auto& cur_qd = cur_js_conv_.getVelocitiesKDL();
  if (pid.CartToJnt(cur_q, cur_qd, tar_p, tar_v, a_ff, f_ext) < 0)
  {
    rosError(name_, "Cartesian PID failed: " << pid.errorMessage());
    return -1;
  }
  efforts = pid.getEfforts();

  return 0;
}

void EffortControllerRos::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void EffortControllerRos::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  odom_ = odom;
}

void EffortControllerRos::currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js)
{
  cur_js_ = cur_js;

  if (tar_js_ == nullptr && tar_cs_ == nullptr)
    return;

  // JointState -> JntArray
  if (cur_js_conv_.jointStateToJntArray(*cur_js_) < 0)
  {
    rosError(
      name_, "Failed to convert current JointState to Jntarray: " << cur_js_conv_.errorMessage());
    return;
  }

  // Joint space control or Task space control
  JntArray efforts;
  if (tar_js_ != nullptr)
  {
    if (jointSpaceControl(efforts) < 0)
      return;
  }
  else if (tar_cs_ != nullptr)
  {
    if (taskSpaceControl(efforts) < 0)
      return;
  }
  else
  {
    rosError(name_, "Both target joint state and target cartesian state are NULL.");
    return;
  }

  // JntArray -> JointState
  if (cur_js_conv_.jntArrayToJointState(jntarraynull_, jntarraynull_, efforts) < 0)
  {
    rosError(name_, "Failed to convert Jntarray to JointState: " << cur_js_conv_.errorMessage());
    return;
  }

  // コマンドを発行
  auto efforts_msg = boost::make_shared<tobas_msgs::JointEfforts>();
  efforts_msg->name = cur_js_conv_.getNamesMsg();
  efforts_msg->data = cur_js_conv_.getEffortsMsg();
  efforts_pub_.publish(efforts_msg);
}

void EffortControllerRos::targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js)
{
  tar_js_ = tar_js;
  tar_cs_ = nullptr;
}

void EffortControllerRos::targetCartStateCb(const tobas_msgs::CartesianStateConstPtr& tar_cs)
{
  if (odom_ == nullptr)
    return;

  const auto np = tar_cs->name.size();  // The number of endpoints
  if (
    tar_cs->frame_id.size() != np || tar_cs->pose.size() != np || tar_cs->twist.size() != np
    || tar_cs->accel.size() != np || tar_cs->wrench.size() != np)
  {
    rosError(name_, "Cartesian state size mismatch.");
    return;
  }

  tar_cs_ = tar_cs;
  tar_js_ = nullptr;
}

void EffortControllerRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  bool success = true;

  // Joint space control
  if (!pid_.setStiffness(cfg.joint_stiffness))
  {
    rosError(name_, "Failed to set joint stiffness.");
    success = false;
  }

  if (!pid_.setDamping(cfg.joint_damping))
  {
    rosError(name_, "Failed to set joint damping.");
    success = false;
  }

  // Task space control
  if (cfg.linear_stiffness < 0)
  {
    rosError(name_, "Linear stiffness must be non negative.");
    success = false;
  }
  if (cfg.angular_stiffness < 0)
  {
    rosError(name_, "Angular stiffness must be non negative.");
    success = false;
  }
  if (cfg.linear_damping < 0)
  {
    rosError(name_, "Linear damping must be non negative.");
    success = false;
  }
  if (cfg.angular_damping < 0)
  {
    rosError(name_, "Angular damping must be non negative.");
    success = false;
  }

  if (success)
  {
    lin_kp_.fill(cfg.linear_stiffness);
    ang_kp_.fill(cfg.angular_stiffness);
    lin_kd_.fill(cfg.linear_damping);
    ang_kd_.fill(cfg.angular_damping);
    rosInfo(name_, "Dynamic parameters are updated.");
  }
  else
  {
    rosError(name_, "Failed to update dynamic parameters.");
  }
}
}  // namespace tobas_manipulation
