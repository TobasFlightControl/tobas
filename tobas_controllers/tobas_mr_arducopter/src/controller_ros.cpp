#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/math.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_kdl/quaternion.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Throttles.h>

#include "../include/tobas_mr_arducopter/controller_ros.hpp"
#include "../include/tobas_mr_arducopter/packets.hpp"
#include "../include/tobas_mr_arducopter/constants.hpp"

using namespace std;
using namespace KDL;

namespace tobas_mr_arducopter
{
ControllerRos::ControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  initializeSockets();

  registerPublishers();
  registerSubscribers();
}

void ControllerRos::getRosParams()
{
  tobas_ros::getParam(nh_, nh_.getNamespace() + kArduCopterNS + "/channels", channels_);
  if (channels_.size() > kMaxMotors)
    ROS_THROW_NAMED(name_, "Too many rotors. The maximum number is " << kMaxMotors << ".");
  if (!tobas_std::isUnique(channels_))
    ROS_THROW_NAMED(name_, "channels are not unique.");
}

void ControllerRos::registerPublishers()
{
  throttles_pub_ = nh_.advertise<tobas_msgs::Throttles>(tobas::kThrottlesCmdTopic, 1);
}

void ControllerRos::registerSubscribers()
{
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
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
  const size_t wait_ms = ardupilot_online_ ? 1000 : 1;
  ServoPacket pkt;
  ssize_t recv_size = socket_in_.recv(&pkt, sizeof(ServoPacket), wait_ms);

  // Drain the socket in the case we're backed up
  int counter = 0;
  ServoPacket last_pkt;
  while (nh_.ok())
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
                                                      << kMaxConnectionTimeoutCount << "]");
      if (++connection_timeout_count_ > kMaxConnectionTimeoutCount)
      {
        connection_timeout_count_ = 0;
        ardupilot_online_ = false;
        rosWarn(name_, "Broken ArduPilot connection.");
      }
    }
    return;
  }

  const ssize_t expected_pkt_size = sizeof(pkt.motorSpeed[0]) * channels_.size();
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
  const auto throttles = boost::make_shared<tobas_msgs::Throttles>();
  throttles->header.stamp = imu_time;
  throttles->data.resize(channels_.size(), 0.);

  // Fill throttle
  for (size_t i = 0; i < channels_.size(); ++i)
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

void ControllerRos::sendState(const tobas_msgs::Odometry& odom)
{
  FdmPacket pkt;

  // Timestamp [sec]
  pkt.timestamp = odom.header.stamp.toSec();

  // Linear acceleration (Local)
  const auto grav_B_nwu = odom.frame.M.inverse(Vector(0, 0, tobas::kGravity));
  const auto acc_B_ned = R_nwu_ned_.inverse(odom.accel.linear + grav_B_nwu);
  pkt.imuLinearAccelerationXYZ[0] = acc_B_ned.x();
  pkt.imuLinearAccelerationXYZ[1] = acc_B_ned.y();
  pkt.imuLinearAccelerationXYZ[2] = acc_B_ned.z();

  // Angular velocity (Local)
  const auto gyro_B_ned = R_nwu_ned_.inverse(odom.twist.rot);
  pkt.imuAngularVelocityRPY[0] = gyro_B_ned.x();
  pkt.imuAngularVelocityRPY[1] = gyro_B_ned.y();
  pkt.imuAngularVelocityRPY[2] = gyro_B_ned.z();

  // Position (Global)
  const auto pos_W_ned = R_nwu_ned_.inverse(odom.frame.p);
  pkt.positionXYZ[0] = pos_W_ned.x();
  pkt.positionXYZ[1] = pos_W_ned.y();
  pkt.positionXYZ[2] = pos_W_ned.z();

  // Orientation (Global)
  const auto rot_ned = R_nwu_ned_.inverse() * odom.frame.M * R_nwu_ned_;
  const Quaternion quat_ned(rot_ned);
  pkt.imuOrientationQuat[0] = quat_ned.w;
  pkt.imuOrientationQuat[1] = quat_ned.x;
  pkt.imuOrientationQuat[2] = quat_ned.y;
  pkt.imuOrientationQuat[3] = quat_ned.z;

  // Linear velocity (Global)
  const auto vel_W_nwu = odom.frame.M * odom.twist.vel;
  const auto vel_W_ned = R_nwu_ned_.inverse(vel_W_nwu);
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

void ControllerRos::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  receiveAndPublishMotorCommand(odom->header.stamp);
  sendState(*odom);
}
}  // namespace tobas_mr_arducopter
