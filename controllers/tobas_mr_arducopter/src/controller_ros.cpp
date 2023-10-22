#include <dh_std_tools/vector.hpp>
#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_kdl/quaternion.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Throttles.h>

#include "../include/tobas_mr_arducopter/controller_ros.hpp"
#include "../include/tobas_mr_arducopter/packets.hpp"
#include "../include/tobas_mr_arducopter/constants.hpp"

using namespace std;
using namespace KDL;

namespace tobas_mr_arducopter
{
ControllerRos::ControllerRos(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name), server_(pnh_)
{
  getRosParams();
  initializeSockets();

  param_set_msg_.request.value.integer = 0;
  param_set_sc_ = nh_.serviceClient<mavros_msgs::ParamSet>(kParamSetSrvName);
  if (!param_set_sc_.waitForExistence())
    ROS_THROW_NAMED(name_, "Failed to connect to '" << kParamSetSrvName << "' service server.");

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ControllerRos::getRosParams()
{
  dh_ros::getParam(
    pnh_, "max_connection_timeout_count", max_connection_timeout_count_,
    kDefaultMaxConnectionTimeoutCount);

  dh_ros::getParam(pnh_, "num_rotors", num_rotors_);
  if (num_rotors_ > kMaxMotors)
    ROS_THROW_NAMED(name_, "Too many rotors. The maximum number is " << kMaxMotors << ".");

  dh_ros::getParam(pnh_, "channels", channels_);
  if (!dh_std::isUnique(channels_))
    ROS_THROW_NAMED(name_, "channels are not unique.");
}

void ControllerRos::registerPublishers()
{
  throttles_pub_ = nh_.advertise<tobas_msgs::Throttles>(tobas::kThrottlesCmdTopic, 1);
}

void ControllerRos::registerSubscribers()
{
  event_sub_ = nh_.subscribe(tobas::kEventTopic, 1, &self::eventCb, this, tcpNoDelay());
  pt_sub_ = nh_.subscribe(tobas::kPoseTwistTopic, 1, &self::poseTwistCb, this, tcpNoDelay());
}

void ControllerRos::initializeSockets()
{
  if (!socket_in_.bind(kFdmAddr, kFdmPortIn))
    ROS_THROW_NAMED(name_, "failed to bind with " << kFdmAddr << ":" << kFdmPortIn << ".");

  if (!socket_out_.connect(kFdmAddr, kFdmPortOut))
    ROS_THROW_NAMED(name_, "failed to bind with " << kFdmAddr << ":" << kFdmPortOut << ".");
}

void ControllerRos::receiveAndPublishMotorCommand(const ros::Time& imu_time)
{
  // If ArduPilot is online, increase timeout for receive once we detect a packet from FCS.
  // Otherwise skip quickly and do not set control force.
  const uint32_t wait_ms = ardupilot_online_ ? 1000 : 1;
  ServoPacket pkt;
  ssize_t recv_size = socket_in_.recv(&pkt, sizeof(ServoPacket), wait_ms);

  // Drain the socket in the case we're backed up
  int counter = 0;
  ServoPacket last_pkt;
  while (true)
  {
    const ssize_t recv_size_last = socket_in_.recv(&last_pkt, sizeof(ServoPacket), 0);
    if (recv_size_last == -1)
    {
      break;
    }
    ++counter;
    pkt = last_pkt;
    recv_size = recv_size_last;
  }

  if (recv_size == -1)  // Didn't receive a packet
  {
    if (ardupilot_online_)
    {
      rosWarn(
        name_, "Broken ArduPilot connection, count [" << connection_timeout_count_ << "/"
                                                      << max_connection_timeout_count_ << "]");
      if (++connection_timeout_count_ > max_connection_timeout_count_)
      {
        connection_timeout_count_ = 0;
        ardupilot_online_ = false;
        rosWarn(name_, "Broken ArduPilot connection.");
      }
    }
    return;
  }

  const ssize_t expected_pkt_size = sizeof(pkt.motorSpeed[0]) * num_rotors_;
  if (recv_size < expected_pkt_size)
  {
    rosError(
      name_, "Got less than model needs. Got: " << recv_size << "commands, expected size: "
                                                << expected_pkt_size);
  }
  const ssize_t recv_channels = recv_size / sizeof(pkt.motorSpeed[0]);

  if (!ardupilot_online_)
  {
    // Made connection, set some flags
    connection_timeout_count_ = 0;
    ardupilot_online_ = true;
  }

  // Create throttle command message
  auto throttles = boost::make_shared<tobas_msgs::Throttles>();
  throttles->header.stamp = imu_time;
  throttles->data.resize(num_rotors_, 0.);

  // Fill throttle
  for (uint32_t i = 0; i < num_rotors_; ++i)
  {
    if (channels_[i] < recv_channels)
    {
      throttles->data[i] = clamp(pkt.motorSpeed[channels_[i]], 0.0f, 1.0f);
    }
    else
    {
      rosError(
        name_, "control[" << i << "] channel[" << channels_[i]
                          << "] is greater than incoming commands size[" << recv_channels
                          << "], control not applied.");
    }
  }

  // Publish throttle command message
  throttles_pub_.publish(throttles);
}

void ControllerRos::sendState(const tobas_msgs::PoseTwist& pt)
{
  FdmPacket pkt;

  // Timestamp [sec]
  pkt.timestamp = pt.header.stamp.toSec();

  // Linear acceleration (Local)
  const auto grav_B_nwu = pt.pose.euler.Inverse(Vector(0, 0, tobas::kGravity));
  const auto acc_B_ned = R_nwu_ned_.Inverse(pt.accel.linear + grav_B_nwu);
  pkt.imuLinearAccelerationXYZ[0] = acc_B_ned.x();
  pkt.imuLinearAccelerationXYZ[1] = acc_B_ned.y();
  pkt.imuLinearAccelerationXYZ[2] = acc_B_ned.z();

  // Angular velocity (Local)
  const auto gyro_B_ned = R_nwu_ned_.Inverse(pt.twist.rot);
  pkt.imuAngularVelocityRPY[0] = gyro_B_ned.x();
  pkt.imuAngularVelocityRPY[1] = gyro_B_ned.y();
  pkt.imuAngularVelocityRPY[2] = gyro_B_ned.z();

  // Position (Global)
  const auto pos_W_ned = R_nwu_ned_.Inverse(pt.pose.pos);
  pkt.positionXYZ[0] = pos_W_ned.x();
  pkt.positionXYZ[1] = pos_W_ned.y();
  pkt.positionXYZ[2] = pos_W_ned.z();

  // Orientation (Global)
  const auto rot_ned = R_nwu_ned_.Inverse() * pt.pose.euler.toRotation() * R_nwu_ned_;
  const Quaternion quat_ned(rot_ned);
  pkt.imuOrientationQuat[0] = quat_ned.w;
  pkt.imuOrientationQuat[1] = quat_ned.x;
  pkt.imuOrientationQuat[2] = quat_ned.y;
  pkt.imuOrientationQuat[3] = quat_ned.z;

  // Linear velocity (Global)
  const auto vel_W_nwu = pt.pose.euler * pt.twist.vel;
  const auto vel_W_ned = R_nwu_ned_.Inverse(vel_W_nwu);
  pkt.velocityXYZ[0] = vel_W_ned.x();
  pkt.velocityXYZ[1] = vel_W_ned.y();
  pkt.velocityXYZ[2] = vel_W_ned.z();

  // send packet
  socket_out_.send(&pkt, sizeof(pkt));

  // cout << "Linear acceleration: " << acc_B_ned << endl;
  // cout << "Angular velocity: " << gyro_B_ned << endl;
  // cout << "Position: " << pos_W_ned << endl;
  // cout << "Orientation: " << quat_ned << endl;
  // cout << "Linear velocity: " << vel_W_ned << endl;
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
  receiveAndPublishMotorCommand(pt->header.stamp);
  sendState(*pt);
}

void ControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  param_id_ = "ATC_ANG_RLL_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_ANG_RLL_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_ANG_RLL_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_ANG_PIT_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_ANG_PIT_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_ANG_PIT_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_ANG_YAW_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_ANG_YAW_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_ANG_YAW_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_RLL_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_RLL_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_RLL_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_PIT_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_PIT_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_PIT_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_YAW_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_YAW_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_YAW_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_RLL_I";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_RLL_I))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_RLL_I;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_PIT_I";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_PIT_I))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_PIT_I;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_YAW_I";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_YAW_I))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_YAW_I;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_RLL_D";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_RLL_D))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_RLL_D;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_PIT_D";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_PIT_D))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_PIT_D;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_YAW_D";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_YAW_D))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_YAW_D;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_RLL_FF";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_RLL_FF))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_RLL_FF;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_PIT_FF";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_PIT_FF))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_PIT_FF;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_YAW_FF";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_YAW_FF))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_YAW_FF;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_POSZ_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_POSZ_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_POSZ_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELZ_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELZ_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELZ_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELZ_I";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELZ_I))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELZ_I;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELZ_D";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELZ_D))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELZ_D;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELZ_FF";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELZ_FF))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELZ_FF;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_ACCZ_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_ACCZ_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_ACCZ_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_ACCZ_I";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_ACCZ_I))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_ACCZ_I;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_ACCZ_D";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_ACCZ_D))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_ACCZ_D;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_ACCZ_FF";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_ACCZ_FF))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_ACCZ_FF;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_POSXY_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_POSXY_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_POSXY_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELXY_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELXY_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELXY_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELXY_I";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELXY_I))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELXY_I;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELXY_D";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELXY_D))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELXY_D;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELXY_FF";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELXY_FF))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELXY_FF;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  rosInfo(name_, "Dynamic parameters are updated.");
}
}  // namespace tobas_mr_arducopter
