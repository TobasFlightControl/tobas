#include "../../include/plugins/rotor_plugin.hpp"
#include "../../include/multirotor_gazebo_plugins/conversions.hpp"

using namespace std;
using namespace ignition::math;

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
  parent_link_ = link_->GetParentJointsLinks()[0];

  // Initialize the first order filter
  rotor_speed_filter_.initialize(time_const_up_, time_const_down_, ref_motor_input_);

  // Advertise
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
  if (!getSdfParam<string>(sdf, "robotNamespace", ns_))
  {
    gzthrow(kPluginName << ": Please specify robotNamespace.");
  }

  if (!getSdfParam<string>(sdf, "linkName", link_name_))
  {
    gzthrow(kPluginName << ": Please specify linkName of the rotor.");
  }

  if (!getSdfParam<string>(sdf, "jointName", joint_name_))
  {
    gzthrow(kPluginName << ": Please specify jointName, where the rotor is attached.");
  }

  if (!getSdfParam<int>(sdf, "motorNumber", motor_number_))
  {
    gzthrow(kPluginName << ": Please specify motorNumber.");
  }

  if (sdf->HasElement("turningDirection"))
  {
    string turning_direction = sdf->GetElement("turningDirection")->Get<string>();
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

  if (!getSdfParam<double>(sdf, "maxRotVelocity", max_rot_vel_))
  {
    gzthrow(kPluginName << ": Please specify maxRotVelocity [rad/s].");
  }
  if (max_rot_vel_ < 0.)
  {
    gzthrow(kPluginName << ": Invalid maxRotVelocity: " << max_rot_vel_ << " [rad/s]");
  }

  if (!getSdfParam<double>(sdf, "motorConstant", motor_const_))
  {
    gzthrow(kPluginName << ": Please specify motorConstant [kg*m/s^2]");
  }
  if (motor_const_ < 0.)
  {
    gzthrow(kPluginName << ": Invalid motorConstant: " << motor_const_ << " [kg*m/s^2]");
  }

  if (!getSdfParam<double>(sdf, "momentConstant", moment_const_))
  {
    gzthrow(kPluginName << ": Please specify momentConstant [m]");
  }
  if (moment_const_ < 0.)
  {
    gzthrow(kPluginName << ": Invalid momentConstant:" << moment_const_ << " [m]");
  }

  if (!getSdfParam<double>(sdf, "rotorDragCoefficient", rotor_drag_coef_))
  {
    gzthrow(kPluginName << ": Please specify rotorDragCoefficient [Ns^2/m^2]");
  }
  if (rotor_drag_coef_ < 0.)
  {
    gzthrow(kPluginName << ": Invalid rotorDragCoefficient:" << rotor_drag_coef_ << " [Ns^2/m^2]");
  }

  if (!getSdfParam<double>(sdf, "timeConstantUp", time_const_up_))
  {
    gzthrow(kPluginName << ": Please specify timeConstantUp [s]");
  }
  if (time_const_up_ <= 0.)
  {
    gzthrow(kPluginName << ": Invalid timeConstantUp:" << time_const_up_ << " [s]");
  }

  if (!getSdfParam<double>(sdf, "timeConstantDown", time_const_down_))
  {
    gzthrow(kPluginName << ": Please specify timeConstantDown [s]");
  }
  if (time_const_down_ <= 0.)
  {
    gzthrow(kPluginName << ": Invalid timeConstantDown:" << time_const_down_ << " [s]");
  }

  getSdfParam<string>(sdf, "motorSpeedPubTopic", motor_speed_pub_topic_, kDefaultSpeedPubTopic);
  getSdfParam<string>(sdf, "commandSubTopic", cmd_sub_topic_, kDefaultCmdSubTopic);
  getSdfParam<string>(sdf, "windSpeedSubTopic", wind_speed_sub_topic_, kDefaultWindSubTopic);

  if (!getSdfParam<double>(sdf, "rotorVelocitySlowdownSim", rotor_speed_slowdown_sim_))
  {
    gzlog << kPluginName << ": rotorVelocitySlowdownSim is not specified. The default value "
          << kDefaultRotorSpeedSlowdownSim << " is used." << endl;
    rotor_speed_slowdown_sim_ = kDefaultRotorSpeedSlowdownSim;
  }
  if (rotor_speed_slowdown_sim_ < 1.)
  {
    gzerr << kPluginName << ": Invalid rotorVelocitySlowdownSim: " << rotor_speed_slowdown_sim_
          << ". The default value " << kDefaultRotorSpeedSlowdownSim << " is used." << endl;
    rotor_speed_slowdown_sim_ = kDefaultRotorSpeedSlowdownSim;
  }
}

void GazeboRotorPlugin::onUpdate(const common::UpdateInfo& info)
{
  double dt = info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = info.simTime.Double();
  updateForcesAndMoments(dt);

  motor_speed_msg_.data = joint_->GetVelocity(0) * rotor_speed_slowdown_sim_;  // real speed
  motor_speed_pub_.publish(motor_speed_msg_);
}

void GazeboRotorPlugin::updateForcesAndMoments(double dt)
{
  double rot_vel_sim = joint_->GetVelocity(0);
  if (abs(rot_vel_sim) * dt > M_PI)
  {
    gzerr << kPluginName << ": Aliasing on motor [" << motor_number_
          << "] might occur. Lower simulation time step or raise rotorVelocitySlowdownSim." << endl;
    return;
  }

  // The True Role of Accelerometer Feedback in Quadrotor Control [Martin+, 2010]
  // II-A. Model of a single propeller near hovering
  // TODO: Implement other terms
  // TODO: II-B. Model of the complete quadrotor

  // (1) first term: Thrust Force
  double rot_vel_real = rot_vel_sim * rotor_speed_slowdown_sim_;
  int rot_vel_sgn = (rot_vel_real > 0.) - (rot_vel_real < 0.);
  double thrust = direction_ * rot_vel_sgn * motor_const_ * sqr(rot_vel_real);  // [N]

  // (1) second term: H-force
  Vector3d joint_axis = joint_->GlobalAxis(0);
  Vector3d body_vel_W = link_->WorldLinearVel();
  Vector3d relative_wind_vel_W = body_vel_W - wind_speed_W_;
  Vector3d body_vel_perp = relative_wind_vel_W - (relative_wind_vel_W.Dot(joint_axis) * joint_axis);
  Vector3d air_drag = -abs(rot_vel_real) * rotor_drag_coef_ * body_vel_perp;

  // (2) first term: Rotor drag torque
  Pose3d pose_diff = link_->WorldCoGPose() - parent_link_->WorldCoGPose();
  Vector3d drag_torque(0., 0., -direction_ * thrust * moment_const_);             // self frame
  Vector3d drag_torque_parent_frame = pose_diff.Rot().RotateVector(drag_torque);  // parent frame

  // For debug
  // cout << "Thrust force: " << thrust << " [N]" << endl;
  // cout << "H force: " << air_drag.Length() << " [N]" << endl;
  // cout << "Rotor drag torque: " << drag_torque.Length() << " [Nm]" << endl;
  // cout << endl;

  // Apply forces and torques
  link_->AddRelativeForce(Vector3d(0., 0., thrust));
  link_->AddForce(air_drag);
  parent_link_->AddRelativeTorque(drag_torque_parent_frame);

  // Apply the filter on the motor velocity
  double ref_motor_rot_vel = rotor_speed_filter_.updateFilter(ref_motor_input_, dt);
  joint_->SetVelocity(0, direction_ * ref_motor_rot_vel / rotor_speed_slowdown_sim_);
}

void GazeboRotorPlugin::commandCb(const CmdMsg& cmd)
{
  if (motor_number_ > cmd.speeds.size() - 1)
  {
    gzerr << kPluginName << ": You tried to access index " << motor_number_
          << " of the RotorSpeeds message array which is of size " << cmd.speeds.size() << endl;
    return;
  }

  ref_motor_input_ = min(cmd.speeds[motor_number_], max_rot_vel_);
}

void GazeboRotorPlugin::windSpeedCb(const WindMsg& wind)
{
  // TODO: Transform velocity to world frame if frame_id is set to something else.
  vectorRosToGazebo(wind.velocity, wind_speed_W_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRotorPlugin);
}  // namespace gazebo
