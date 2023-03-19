#include "../../include/multirotor_gazebo_plugins/magnetometer_plugin.hpp"
#include "../../include/multirotor_gazebo_plugins/utils.hpp"

#define ZERO_3 (SdfVector3(0., 0., 0.))

using namespace std;

namespace gazebo
{
GazeboMagnetometerPlugin::GazeboMagnetometerPlugin() : ModelPlugin(), rnd_gen_(rnd_dev_())
{
}

void GazeboMagnetometerPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get SDF parameters
  getSdfParams(sdf);

  // Store the pointer to the model and the world
  model_ = model;
  world_ = model_->GetWorld();

  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
  {
    gzthrow("[gazebo_magnetometer_plugin] Couldn't find specified link \"" << link_name_ << "\".");
  }

  // Create the normal noise distributions
  noise_[0] = NormalDistribution(0, noise_normal_.X());
  noise_[1] = NormalDistribution(0, noise_normal_.Y());
  noise_[2] = NormalDistribution(0, noise_normal_.Z());

  // Create the uniform noise distribution for initial bias
  UniformDistribution initial_bias[3];
  initial_bias[0] =
    UniformDistribution(-noise_uniform_initial_bias_.X(), noise_uniform_initial_bias_.X());
  initial_bias[1] =
    UniformDistribution(-noise_uniform_initial_bias_.Y(), noise_uniform_initial_bias_.Y());
  initial_bias[2] =
    UniformDistribution(-noise_uniform_initial_bias_.Z(), noise_uniform_initial_bias_.Z());

  // Initialize the reference magnetic field vector in NWU world frame
  mag_NWU_ = ignition::math::Vector3d(
    ref_mag_north_ + initial_bias[0](rnd_gen_), -ref_mag_east_ + initial_bias[1](rnd_gen_),
    -ref_mag_down_ + initial_bias[2](rnd_gen_));

  // Fill the static parts of the magnetometer message
  mag_msg_.header.frame_id = link_name_;

  mag_msg_.magnetic_field_covariance[0] = sqr(noise_normal_.X());
  mag_msg_.magnetic_field_covariance[1] = 0.;
  mag_msg_.magnetic_field_covariance[2] = 0.;
  mag_msg_.magnetic_field_covariance[3] = 0.;
  mag_msg_.magnetic_field_covariance[4] = sqr(noise_normal_.Y());
  mag_msg_.magnetic_field_covariance[5] = 0.;
  mag_msg_.magnetic_field_covariance[6] = 0.;
  mag_msg_.magnetic_field_covariance[7] = 0.;
  mag_msg_.magnetic_field_covariance[8] = sqr(noise_normal_.Z());

  // Advertise publisher
  mag_pub_ = nh_.advertise<MagMsg>("/" + ns_ + "/" + mag_topic_, 1);

  // Listen to the update event. This event is broadcast every simulation iteration
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboMagnetometerPlugin::OnUpdate, this, _1));
}

void GazeboMagnetometerPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  if (sdf->HasElement("robotNamespace"))
  {
    ns_ = sdf->GetElement("robotNamespace")->Get<string>();
  }
  else
  {
    gzerr << "[gazebo_magnetometer_plugin] Please specify a robotNamespace." << endl;
  }

  if (sdf->HasElement("linkName"))
  {
    link_name_ = sdf->GetElement("linkName")->Get<string>();
  }
  else
  {
    gzerr << "[gazebo_magnetometer_plugin] Please specify a linkName." << endl;
  }

  getSdfParam<string>(sdf, "magnetometerTopic", mag_topic_, defaultMagTopic);
  getSdfParam<double>(sdf, "refMagNorth", ref_mag_north_, defaultRefMagNorth);
  getSdfParam<double>(sdf, "refMagEast", ref_mag_east_, defaultRefMagEast);
  getSdfParam<double>(sdf, "refMagDown", ref_mag_down_, defaultRefMagDown);
  getSdfParam<SdfVector3>(sdf, "noiseNormal", noise_normal_, ZERO_3);
  getSdfParam<SdfVector3>(sdf, "noiseUniformInitialBias", noise_uniform_initial_bias_, ZERO_3);
}

void GazeboMagnetometerPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Get the current pose and time from Gazebo
  ignition::math::Pose3d T_W_B = link_->WorldPose();
  common::Time cur_time = world_->SimTime();

  // Calculate the magnetic field noise
  ignition::math::Vector3d mag_noise(noise_[0](rnd_gen_), noise_[1](rnd_gen_), noise_[2](rnd_gen_));

  // Rotate the earth magnetic field into the inertial frame
  ignition::math::Vector3d field_B = T_W_B.Rot().RotateVectorReverse(mag_NWU_ + mag_noise);

  // Fill the magnetic field message
  mag_msg_.header.stamp.sec = cur_time.sec;
  mag_msg_.header.stamp.nsec = cur_time.nsec;
  mag_msg_.magnetic_field.x = field_B.X();
  mag_msg_.magnetic_field.y = field_B.Y();
  mag_msg_.magnetic_field.z = field_B.Z();

  // Publish the message
  mag_pub_.publish(mag_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMagnetometerPlugin);
}  // namespace gazebo
