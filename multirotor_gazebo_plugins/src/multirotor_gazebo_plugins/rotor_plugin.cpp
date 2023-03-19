#include "../../include/multirotor_gazebo_plugins/rotor_plugin.hpp"

#define CW -1
#define CCW 1

using namespace std;

namespace gazebo
{
GazeboRotorPlugin::GazeboRotorPlugin()
  : ModelPlugin(), ref_motor_input_(0.), prev_sim_time_(0.), wind_speed_W_(0., 0., 0.)
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

  // Initialize the first order filter
  rotor_speed_filter_.initialize(time_const_up_, time_const_down_, ref_motor_input_);

  // Advertise and register
  motor_speed_pub_ = nh_.advertise<std_msgs::Float64>("/" + ns_ + "/" + motor_speed_pub_topic_, 1);
  command_sub_ =
    nh_.subscribe("/" + ns_ + "/" + cmd_sub_topic_, 1, &GazeboRotorPlugin::commandCb, this);
  wind_speed_sub_ = nh_.subscribe(
    "/" + ns_ + "/" + wind_speed_sub_topic_, 1, &GazeboRotorPlugin::windSpeedCb, this);

  // Listen to the update event
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRotorPlugin::onUpdate, this, _1));
}

void GazeboRotorPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  if (sdf->HasElement("robotNamespace"))
  {
    ns_ = sdf->GetElement("robotNamespace")->Get<string>();
  }
  else
  {
    gzthrow(kPluginName << ": Please specify a robotNamespace.");
  }

  if (sdf->HasElement("linkName"))
  {
    link_name_ = sdf->GetElement("linkName")->Get<string>();
  }
  else
  {
    gzthrow(kPluginName << ": Please specify a linkName of the rotor.");
  }

  if (sdf->HasElement("jointName"))
  {
    joint_name_ = sdf->GetElement("jointName")->Get<string>();
  }
  else
  {
    gzthrow(kPluginName << ": Please specify a jointName, where the rotor is attached.");
  }

  if (sdf->HasElement("motorNumber"))
  {
    motor_number_ = sdf->GetElement("motorNumber")->Get<int>();
  }
  else
  {
    gzthrow(kPluginName << ": Please specify a motorNumber.");
  }

  if (sdf->HasElement("turningDirection"))
  {
    string turning_direction = sdf->GetElement("turningDirection")->Get<string>();
    if (turning_direction == "cw")
    {
      direction_ = CW;
    }
    else if (turning_direction == "ccw")
    {
      direction_ = CCW;
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

  // TODO: 範囲チェック
  getSdfParam<string>(sdf, "motorSpeedPubTopic", motor_speed_pub_topic_, kDefaultSpeedPubTopic);
  getSdfParam<string>(sdf, "commandSubTopic", cmd_sub_topic_, kDefaultCmdSubTopic);
  getSdfParam<string>(sdf, "windSpeedSubTopic", wind_speed_sub_topic_, kDefaultWindSubTopic);
  getSdfParam<double>(sdf, "maxRotVelocity", max_rot_vel_, kDefaultMaxRotSpeed);
  getSdfParam<double>(sdf, "motorConstant", motor_constant_, kDefaultMotorConst);
  getSdfParam<double>(sdf, "momentConstant", moment_constant_, kDefaultMomentConst);
  getSdfParam<double>(sdf, "rotorDragCoefficient", rotor_drag_coef_, kDefaultRotorDragCoef);
  getSdfParam<double>(sdf, "rollingMomentCoefficient", roll_moment_coef_, kDefaultRollMomentCoef);
  getSdfParam<double>(sdf, "timeConstantUp", time_const_up_, kDefaultTimeConstUp);
  getSdfParam<double>(sdf, "timeConstantDown", time_const_down_, kDefaultTimeConstDown);
  getSdfParam<double>(
    sdf, "rotorVelocitySlowdownSim", rotor_speed_slowdown_sim_, kDefaultRotorSpeedSlowdownSim);
}

void GazeboRotorPlugin::onUpdate(const common::UpdateInfo& info)
{
  double dt = info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = info.simTime.Double();
  updateForcesAndMoments(dt);

  motor_speed_msg_.data = joint_->GetVelocity(0.);
  motor_speed_pub_.publish(motor_speed_msg_);
}

void GazeboRotorPlugin::updateForcesAndMoments(double dt)
{
  double motor_rot_vel = joint_->GetVelocity(0);
  if (motor_rot_vel / (2 * M_PI) > 1 / (2 * dt))
  {
    gzerr << "Aliasing on motor [" << motor_number_
          << "] might occur. Consider making smaller simulation time "
             "steps or raising the rotor_speed_slowdown_sim_ param."
          << endl;
  }
  double real_motor_vel = motor_rot_vel * rotor_speed_slowdown_sim_;
  // Get the direction of the rotor rotation.
  int real_motor_vel_sign = (real_motor_vel > 0.) - (real_motor_vel < 0.);
  // Assuming symmetric propellers (or rotors) for the thrust calculation.
  double thrust = direction_ * real_motor_vel_sign * motor_constant_ * sqr(real_motor_vel);

  // Apply a force to the link.
  link_->AddRelativeForce(ignition::math::Vector3d(0., 0., thrust));

  // Forces from Philppe Martin's and Erwan Salaün's
  // 2010 IEEE Conference on Robotics and Automation paper
  // The True Role of Accelerometer Feedback in Quadrotor Control
  // - \omega * \lambda_1 * V_A^{\perp}
  ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
  ignition::math::Vector3d body_vel_W = link_->WorldLinearVel();
  ignition::math::Vector3d relative_wind_vel_W = body_vel_W - wind_speed_W_;
  ignition::math::Vector3d body_vel_perpendicular =
    relative_wind_vel_W - (relative_wind_vel_W.Dot(joint_axis) * joint_axis);
  ignition::math::Vector3d air_drag =
    -abs(real_motor_vel) * rotor_drag_coef_ * body_vel_perpendicular;

  // Apply air_drag to link.
  link_->AddForce(air_drag);
  // Moments get the parent link, such that the resulting torques can be applied.
  physics::Link_V parent_links = link_->GetParentJointsLinks();
  // The tansformation from the parent_link to the link_.
  ignition::math::Pose3d pose_diff = link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
  ignition::math::Vector3d drag_torque(0., 0., -direction_ * thrust * moment_constant_);
  // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
  ignition::math::Vector3d drag_torque_parent_frame = pose_diff.Rot().RotateVector(drag_torque);
  parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

  ignition::math::Vector3d rolling_moment;
  // - \omega * \mu_1 * V_A^{\perp}
  rolling_moment = -abs(real_motor_vel) * roll_moment_coef_ * body_vel_perpendicular;
  parent_links.at(0)->AddTorque(rolling_moment);
  // Apply the filter on the motor's velocity.
  double ref_motor_rot_vel = rotor_speed_filter_.updateFilter(ref_motor_input_, dt);

  // Make sure max force is set, as it may be reset to 0 by a world reset any
  // time. (This cannot be done during Reset() because the change will be undone
  // by the Joint's reset function afterwards.)
  joint_->SetVelocity(0, direction_ * ref_motor_rot_vel / rotor_speed_slowdown_sim_);
}

void GazeboRotorPlugin::commandCb(const CmdMsg& cmd)
{
  if (motor_number_ > cmd.speeds.size() - 1)
  {
    gzerr << "You tried to access index " << motor_number_
          << " of the MotorSpeed message array which is of size " << cmd.speeds.size();
  }

  ref_motor_input_ = min(cmd.speeds[motor_number_], max_rot_vel_);
}

void GazeboRotorPlugin::windSpeedCb(const WindMsg& wind)
{
  // TODO: Transform velocity to world frame if frame_id is set to something else.
  wind_speed_W_.X() = wind.velocity.x;
  wind_speed_W_.Y() = wind.velocity.y;
  wind_speed_W_.Z() = wind.velocity.z;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRotorPlugin);
}  // namespace gazebo
