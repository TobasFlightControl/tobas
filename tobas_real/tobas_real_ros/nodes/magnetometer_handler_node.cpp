#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/ellipse_transformer.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs_adapter/MagneticField.hpp>
#include <tobas_msgs_adapter/MagneticField.hpp>

#include <tobas_real_common/constants.hpp>

using namespace std;
using namespace real::handler::mag;

class MagnetometerHandlerNode : public tobas::BaseNode
{
  static constexpr double kHpfCutoff = 10.;  // [Hz] (G(1Hz) ~ 0.1, G(20Hz) ~ 0.9)
  static constexpr size_t kWindowSize = 100;

  using self = MagnetometerHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit MagnetometerHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  math::EllipseTransformer mag_trans_;

  tobas_hal_msgs::MagneticField::ConstSharedPtr mag_raw_;
  ptree::PropertyTree pt_;
  std::array<dsp::NoiseVarianceFilter, 3> mag_noise_;

  ros2::PublisherPtr<tobas_msgs::MagneticField> mag_pub_;
  ros2::SubscriberPtr<tobas_hal_msgs::MagneticField> mag_sub_;

  void readConfig();

  bool paramsCb(const std::vector<double>& params);
  void magCb(const tobas_hal_msgs::MagneticField::ConstSharedPtr& mag_raw);
};

MagnetometerHandlerNode::MagnetometerHandlerNode(const rclcpp::NodeOptions& options)
  : super("magnetometer_handler", options)
{
  if (!pt_.initialize(linux::expandUser(kIniPath)))
    TOBAS_EXIT("Failed to initialize property tree.");

  readConfig();

  addDynamicDoubleArrayParam(real::handler::kParamName, &self::paramsCb, this);

  mag_pub_ = createPublisher<tobas_msgs::MagneticField>(tobas::kMagTopic);
  mag_sub_ = createSubscriber(hal::kMagTopic, &self::magCb, this);
}

void MagnetometerHandlerNode::readConfig()
{
  if (!pt_.get(kAxxKey, mag_trans_.a_xx))
  {
    TOBAS_WARN("Failed to get \"", kAxxKey, "\". from configuration file. All params are set to defaults.");
    mag_trans_.setIdentity();
    return;
  }
  if (!pt_.get(kAyyKey, mag_trans_.a_yy))
  {
    TOBAS_WARN("Failed to get \"", kAyyKey, "\". from configuration file. All params are set to defaults.");
    mag_trans_.setIdentity();
    return;
  }
  if (!pt_.get(kAzzKey, mag_trans_.a_zz))
  {
    TOBAS_WARN("Failed to get \"", kAzzKey, "\". from configuration file. All params are set to defaults.");
    mag_trans_.setIdentity();
    return;
  }
  if (!pt_.get(kAxyKey, mag_trans_.a_xy))
  {
    TOBAS_WARN("Failed to get \"", kAxyKey, "\". from configuration file. All params are set to defaults.");
    mag_trans_.setIdentity();
    return;
  }
  if (!pt_.get(kAyzKey, mag_trans_.a_yz))
  {
    TOBAS_WARN("Failed to get \"", kAyzKey, "\". from configuration file. All params are set to defaults.");
    mag_trans_.setIdentity();
    return;
  }
  if (!pt_.get(kAzxKey, mag_trans_.a_zx))
  {
    TOBAS_WARN("Failed to get \"", kAzxKey, "\". from configuration file. All params are set to defaults.");
    mag_trans_.setIdentity();
    return;
  }
  if (!pt_.get(kBxKey, mag_trans_.b_x))
  {
    TOBAS_WARN("Failed to get \"", kBxKey, "\". from configuration file. All params are set to defaults.");
    mag_trans_.setIdentity();
    return;
  }
  if (!pt_.get(kByKey, mag_trans_.b_y))
  {
    TOBAS_WARN("Failed to get \"", kByKey, "\". from configuration file. All params are set to defaults.");
    mag_trans_.setIdentity();
    return;
  }
  if (!pt_.get(kBzKey, mag_trans_.b_z))
  {
    TOBAS_WARN("Failed to get \"", kBzKey, "\". from configuration file. All params are set to defaults.");
    mag_trans_.setIdentity();
    return;
  }
  if (!pt_.get(kCKey, mag_trans_.c))
  {
    TOBAS_WARN("Failed to get \"", kCKey, "\". from configuration file. All params are set to defaults.");
    mag_trans_.setIdentity();
    return;
  }

  if (!mag_trans_.initialize())
  {
    TOBAS_ERROR("Failed to initialize ellipse transformer. All params are set to defaults.");
    mag_trans_.setIdentity();
    return;
  }
}

bool MagnetometerHandlerNode::paramsCb(const std::vector<double>& params)
{
  // Skip first call
  if (params.size() == 0)
    return false;

  // Check size
  if (params.size() != kParamSize)
  {
    TOBAS_ERROR("Parameter size mismatch.");
    return false;
  }

  // Update parameters
  mag_trans_.a_xx = params.at(kAxxChannel);
  mag_trans_.a_yy = params.at(kAyyChannel);
  mag_trans_.a_zz = params.at(kAzzChannel);
  mag_trans_.a_xy = params.at(kAxyChannel);
  mag_trans_.a_yz = params.at(kAyzChannel);
  mag_trans_.a_zx = params.at(kAzxChannel);
  mag_trans_.b_x = params.at(kBxChannel);
  mag_trans_.b_y = params.at(kByChannel);
  mag_trans_.b_z = params.at(kBzChannel);
  mag_trans_.c = params.at(kCChannel);

  // Verify parameters
  if (!mag_trans_.initialize())
  {
    TOBAS_ERROR("Failed to initialize ellipse transformer. Reloading...");
    readConfig();
    return false;
  }

  // Save parameters
  pt_.set(kAxxKey, params.at(kAxxChannel));
  pt_.set(kAyyKey, params.at(kAyyChannel));
  pt_.set(kAzzKey, params.at(kAzzChannel));
  pt_.set(kAxyKey, params.at(kAxyChannel));
  pt_.set(kAyzKey, params.at(kAyzChannel));
  pt_.set(kAzxKey, params.at(kAzxChannel));
  pt_.set(kBxKey, params.at(kBxChannel));
  pt_.set(kByKey, params.at(kByChannel));
  pt_.set(kBzKey, params.at(kBzChannel));
  pt_.set(kCKey, params.at(kCChannel));
  if (pt_.save())
  {
    TOBAS_ERROR("Failed to save parameters.");
    return false;
  }

  return true;
}

void MagnetometerHandlerNode::magCb(const tobas_hal_msgs::MagneticField::ConstSharedPtr& mag_raw)
{
  // Project data to unit sphere
  const auto mag_unit = mag_trans_.transform(mag_raw->magnetic_field.data);

  // Initialize
  if (mag_raw_ == nullptr)
  {
    for (size_t i = 0; i < 3; ++i)
      mag_noise_[i].initialize(kWindowSize, kHpfCutoff, mag_unit(i));
    mag_raw_ = mag_raw;
    return;
  }

  // Compute time difference
  const auto dt = (mag_raw->header.stamp - mag_raw_->header.stamp).seconds();
  mag_raw_ = mag_raw;

  // Update noise filter
  for (size_t i = 0; i < 3; ++i)
    if (mag_noise_[i].update(mag_unit(i), dt) < 0)
      TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Noise filter failed: ", mag_noise_[i].errorMessage());

  // Create message
  auto mag_msg = std::make_unique<tobas_msgs::MagneticField>();

  // Fill header
  mag_msg->header = mag_raw->header;

  // Fill data
  mag_msg->magnetic_field.data = mag_unit;

  // Fill covariance matrices
  mag_msg->covariance.setZero();
  for (size_t i = 0; i < 3; ++i)
    mag_msg->covariance(i, i) = mag_noise_[i].noiseVariance();

  // Publish message
  mag_pub_->publish(move(mag_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(MagnetometerHandlerNode)
