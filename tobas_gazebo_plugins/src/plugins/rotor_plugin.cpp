#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>

#include "../../include/plugins/rotor_plugin.hpp"
#include "../../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../../include/tobas_gazebo_plugins/constants.hpp"
#include "../../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"
#include "../../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboRotorPlugin::GazeboRotorPlugin()
  : super(), ref_rot_speed_(0.), prev_sim_time_(0.), wind_speed_W_(0., 0., 0.)
{
}

void GazeboRotorPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get SDF parameters
  getSdfParams(sdf);

  // Store the pointer to the model
  model_ = model;

  // Get the pointer to the joint and the link
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
  {
    gzthrow(kPluginName << ": Couldn't find specified joint \"" << joint_name_ << "\".");
  }

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
  {
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");
  }
  parent_link_ = link_->GetParentJointsLinks()[0];

  // Initialize the first order filter
  rotor_speed_filter_.initialize(time_const_up_, time_const_down_, ref_rot_speed_);

  registerPubSub();

  // Listen to the update event
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRotorPlugin::onUpdate, this, _1));
}

void GazeboRotorPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "jointName", joint_name_);
  getSdfParam(sdf, "motorNumber", motor_number_, NON_NEGATIVE);

  if (sdf->HasElement("turningDirection"))
  {
    const auto turning_direction = sdf->GetElement("turningDirection")->Get<string>();
    if (turning_direction == "cw")
    {
      direction_ = -1;
    }
    else if (turning_direction == "ccw")
    {
      direction_ = 1;
    }
    else
    {
      gzthrow(kPluginName << ": Please only use 'cw' or 'ccw' as turningDirection.");
    }
  }
  else
  {
    gzthrow(kPluginName << ": Please specify a turning direction ('cw' or 'ccw').");
  }

  getSdfParam(sdf, "maxRotVelocity", max_rot_speed_, NON_NEGATIVE);
  getSdfParam(sdf, "motorConstant", motor_const_, NON_NEGATIVE);
  getSdfParam(sdf, "momentConstant", moment_const_, NON_NEGATIVE);
  getSdfParam(sdf, "rotorDragCoefficient", rotor_drag_coef_, NON_NEGATIVE);
  getSdfParam(sdf, "timeConstantUp", time_const_up_, POSITIVE);
  getSdfParam(sdf, "timeConstantDown", time_const_down_, POSITIVE);

  getSdfParam(sdf, "motorSpeedPubTopic", motor_speed_pub_topic_, kDefaultSpeedPubTopic);
  getSdfParam(sdf, "commandSubTopic", cmd_sub_topic_, kDefaultCmdSubTopic);
  getSdfParam(sdf, "windSpeedSubTopic", wind_speed_sub_topic_, kDefaultWindSubTopic);

  getSdfParam(
    sdf, "rotorVelocitySlowdownSim", rotor_speed_slowdown_sim_, kDefaultRotorSpeedSlowdownSim);
  if (rotor_speed_slowdown_sim_ < 1.)
  {
    gzerr << kPluginName << ": Invalid rotorVelocitySlowdownSim: " << rotor_speed_slowdown_sim_
          << ". The default value " << kDefaultRotorSpeedSlowdownSim << " is used." << endl;
    rotor_speed_slowdown_sim_ = kDefaultRotorSpeedSlowdownSim;
  }

  getSdfParam(sdf, "checkDelayThreshold", check_delay_threshold_, kDefaultCheckDelayThreshold);
  if (check_delay_threshold_ <= 0.)
  {
    gzerr << kPluginName << ": Invalid checkDelayThreshold: " << check_delay_threshold_
          << ". The default value " << kDefaultCheckDelayThreshold << " is used." << endl;
    check_delay_threshold_ = kDefaultCheckDelayThreshold;
  }
}

void GazeboRotorPlugin::onUpdate(const common::UpdateInfo& info)
{
  const auto dt = info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = info.simTime.Double();
  updateForcesAndMoments(dt);

  motor_speed_msg_.data = joint_->GetVelocity(0) * rotor_speed_slowdown_sim_;  // real speed
  motor_speed_pub_.publish(motor_speed_msg_);
}

void GazeboRotorPlugin::registerPubSub()
{
  motor_speed_pub_ = nh_.advertise<std_msgs::Float64>("/" + ns_ + "/" + motor_speed_pub_topic_, 1);

  command_sub_ =
    nh_.subscribe("/" + ns_ + "/" + cmd_sub_topic_, 1, &GazeboRotorPlugin::commandCb, this);
  wind_speed_sub_ = nh_.subscribe(
    "/" + ns_ + "/" + wind_speed_sub_topic_, 1, &GazeboRotorPlugin::windSpeedCb, this);
}

void GazeboRotorPlugin::updateForcesAndMoments(double dt)
{
  const auto rot_speed_sim = joint_->GetVelocity(0);
  if (abs(rot_speed_sim) * dt > M_PI)
  {
    gzerr << kPluginName << ": Aliasing on motor [" << motor_number_
          << "] might occur. Lower simulation time step or raise rotorVelocitySlowdownSim." << endl;
  }

  // The True Role of Accelerometer Feedback in Quadrotor Control [Martin+, 2010]
  // II-A. Model of a single propeller near hovering
  // TODO: Implement other terms
  // TODO: II-B. Model of the complete quadrotor

  // Get joint axes
  const auto global_axis = joint_->GlobalAxis(0);
  const auto local_axis = joint_->LocalAxis(0);

  // (1) first term: Thrust Force
  const auto rot_vel_real = rot_speed_sim * rotor_speed_slowdown_sim_;
  const auto rot_vel_sgn = dh_std::sign(rot_vel_real);
  const auto thrust = direction_ * rot_vel_sgn * motor_const_ * dh_std::sqr(rot_vel_real);
  link_->AddForce(thrust * global_axis);

  // (1) second term: H-force
  const auto linvel_W = link_->WorldLinearVel() - wind_speed_W_;
  const auto linvel_perp = linvel_W - (linvel_W.Dot(global_axis) * global_axis);
  const auto air_drag = -abs(rot_vel_real) * rotor_drag_coef_ * linvel_perp;
  link_->AddForce(air_drag);

  // (2) first term: Rotor drag torque
  const auto pose_diff = link_->WorldCoGPose() - parent_link_->WorldCoGPose();
  const auto drag_torque_child = (-direction_ * thrust * moment_const_) * local_axis;
  const auto drag_torque_parent = pose_diff.Rot().RotateVector(drag_torque_child);
  parent_link_->AddRelativeTorque(drag_torque_parent);

  // For debug
  // cout << "Thrust force: " << thrust << " [N]" << endl;
  // cout << "H force: " << air_drag.Length() << " [N]" << endl;
  // cout << "Rotor drag torque: " << drag_torque.Length() << " [Nm]" << endl;
  // cout << endl;

  // Apply the filter on the motor velocity
  const auto ref_rot_speed = rotor_speed_filter_.updateFilter(ref_rot_speed_, dt);
  joint_->SetVelocity(0, direction_ * ref_rot_speed / rotor_speed_slowdown_sim_);
}

void GazeboRotorPlugin::commandCb(const CmdMsg& cmd)
{
  // Check index
  if (motor_number_ >= cmd.speeds.size())
  {
    gzerr << kPluginName << ": You tried to access index " << motor_number_
          << " of the RotorSpeeds message array which is of size " << cmd.speeds.size() << endl;
    return;
  }

  // Check delay
  const double delay = prev_sim_time_ - cmd.header.stamp.toSec();
  if (delay > check_delay_threshold_)
  {
    gzwarn << kPluginName << ": The delay from sensors to the motor command is " << delay
           << " seconds, which is too large." << endl;
  }
  else if (delay < 0.)
  {
    gzerr << kPluginName << ": The timestamp of the motor command precedes the current time."
          << endl;
  }

  // Get Commanded speed
  const auto cmd_speed = cmd.speeds[motor_number_];

  // Check rotor speed limit
  if (cmd_speed < 0.)
  {
    gzerr << kPluginName << ": The commanded motor speed " << cmd_speed << " is lower than 0."
          << endl;
  }
  else if (cmd_speed > max_rot_speed_)
  {
    gzerr << kPluginName << ": The commanded motor speed " << cmd_speed
          << " exceeds the maximum speed " << max_rot_speed_ << "." << endl;
  }

  // Update reference rotation speed
  ref_rot_speed_ = dh_std::clamp(cmd_speed, 0., max_rot_speed_);
}

void GazeboRotorPlugin::windSpeedCb(const WindMsg& wind)
{
  vectorKDLToGazebo(wind.vel, wind_speed_W_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRotorPlugin);
}  // namespace gazebo
