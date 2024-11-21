#include <tobas_algorithm/kahan.hpp>
#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_dsp/notch_filter.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs_adapter/imu_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/imu_stamped.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

using namespace std;

class ImuPreprocessNode : public tobas::BaseNode
{
  static constexpr double kHpfCutoff = 30.;             // [Hz] (G(3Hz) ~ 0.1, G(100Hz) ~ 0.95)
  static constexpr size_t kWindowSize = 200;            // 400Hzで0.5s
  static constexpr int kMeasureGyroBiasCount = 1000;    // [-]
  static constexpr double kStaticGyroThreshold = 0.5;   // [rad/s]
  static constexpr size_t kNumNotchFilterPerRotor = 2;  // 倍周波の個数

  static constexpr long kDefaultAccelLowPassCutoff = 100;
  static constexpr long kDefaultAccelNotchMinFreq = 25;
  static constexpr long kDefaultAccelNotchQValue = 30;
  static constexpr double kDefaultAccelNotchDepth = 0.;
  static constexpr long kDefaultGyroLowPassCutoff = 100;
  static constexpr long kDefaultGyroNotchMinFreq = 25;
  static constexpr long kDefaultGyroNotchQValue = 30;
  static constexpr double kDefaultGyroNotchDepth = 0.;

  using self = ImuPreprocessNode;
  using super = tobas::BaseNode;

public:
  explicit ImuPreprocessNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum stage_t
  {
    MEASURE_GYRO_BIAS,
    INITIALIZE,
    PUBLISH,
  } stage_ = MEASURE_GYRO_BIAS;

  // rosparam
  double acc_notch_min_freq_ = kDefaultAccelNotchMinFreq;
  double acc_notch_q_ = kDefaultAccelNotchQValue;
  double acc_notch_depth_ = kDefaultAccelNotchDepth;
  double gyro_notch_min_freq_ = kDefaultGyroNotchMinFreq;
  double gyro_notch_q_ = kDefaultGyroNotchQValue;
  double gyro_notch_depth_ = kDefaultGyroNotchDepth;

  // ジャイロバイアス関連
  kdl::Vector gyro_bias_;
  size_t gyro_bias_cnt_ = 0;
  array<algo::Kahan<double>, 3> gyro_sum_;

  tobas_msgs::ImuStamped::ConstSharedPtr imu_raw_;
  dsp::LowPassFilter<kdl::Vector> acc_lpf_, gyro_lpf_;
  map<size_t, size_t> channel2idx_;
  vector<double> rot_speeds_;
  vector<dsp::NotchFilter<kdl::Vector>> acc_notch_, gyro_notch_;
  dsp::NoiseVarianceFilter<double, 3, kWindowSize> acc_noise_, gyro_noise_;

  ros2::PublisherPtr<tobas_msgs::ImuWithCovarianceStamped> imu_pub_;
  ros2::SubscriberPtr<tobas_msgs::ImuStamped> imu_raw_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> rotor_states_sub_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;

  bool updateNotchParams();

  bool accelLowPassCutoffCb(const long& p);
  bool accelNotchMinFreqCb(const long& p);
  bool accelNotchQValueCb(const long& p);
  bool accelNotchDepthCb(const double& p);
  bool gyroLowPassCutoffCb(const long& p);
  bool gyroNotchMinFreqCb(const long& p);
  bool gyroNotchQValueCb(const long& p);
  bool gyroNotchDepthCb(const double& p);

  void imuRawCb(const tobas_msgs::ImuStamped::ConstSharedPtr& imu_raw);
  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& rotor_states);
  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
};

ImuPreprocessNode::ImuPreprocessNode(const rclcpp::NodeOptions& options) : super("imu_preprocess", options)
{
  addDynamicIntParam("accel_lowpass_cutoff", &self::accelLowPassCutoffCb, this, kDefaultAccelLowPassCutoff, 30, 400);
  addDynamicIntParam("accel_notch_min_freq", &self::accelNotchMinFreqCb, this, kDefaultAccelNotchMinFreq, 10, 100);
  addDynamicIntParam("accel_notch_q", &self::accelNotchQValueCb, this, kDefaultAccelNotchQValue, 10, 500);
  addDynamicDoubleParam("accel_notch_depth", &self::accelNotchDepthCb, this, kDefaultAccelNotchDepth, 0., 0.1);
  addDynamicIntParam("gyro_lowpass_cutoff", &self::gyroLowPassCutoffCb, this, kDefaultGyroLowPassCutoff, 30, 400);
  addDynamicIntParam("accel_notch_min_freq", &self::gyroNotchMinFreqCb, this, kDefaultGyroNotchMinFreq, 10, 100);
  addDynamicIntParam("gyro_notch_q", &self::gyroNotchQValueCb, this, kDefaultGyroNotchQValue, 10, 500);
  addDynamicDoubleParam("gyro_notch_depth", &self::gyroNotchDepthCb, this, kDefaultGyroNotchDepth, 0., 0.1);

  imu_pub_ = createPublisher<tobas_msgs::ImuWithCovarianceStamped>(tobas::kImuTopic);
  imu_raw_sub_ = createSubscriber(tobas::kImuRawTopic, &self::imuRawCb, this);
  rotor_states_sub_ = createSubscriber(tobas::kRotorStatesTopic, &self::rotorStatesCb, this);
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this);
}

bool ImuPreprocessNode::updateNotchParams()
{
  for (auto& notch : acc_notch_)
  {
    if (!notch.setQValue(acc_notch_q_))
    {
      TOBAS_ERROR("Failed to set Q value of accel notch filter.");
      return false;
    }
    if (!notch.setDepth(acc_notch_depth_))
    {
      TOBAS_ERROR("Failed to set depth of accel notch filter.");
      return false;
    }
  }

  for (auto& notch : gyro_notch_)
  {
    if (!notch.setQValue(gyro_notch_q_))
    {
      TOBAS_ERROR("Failed to set Q value of gyro notch filter.");
      return false;
    }
    if (!notch.setDepth(gyro_notch_depth_))
    {
      TOBAS_ERROR("Failed to set depth of gyro notch filter.");
      return false;
    }
  }

  return true;
}

bool ImuPreprocessNode::accelLowPassCutoffCb(const long& p)
{
  if (!acc_lpf_.setCutoffFrequency(p))
  {
    TOBAS_ERROR("Failed to set cutoff frequency of accel low-pass filter.");
    return false;
  }

  return true;
}

bool ImuPreprocessNode::accelNotchMinFreqCb(const long& p)
{
  acc_notch_min_freq_ = p;
  return true;
}

bool ImuPreprocessNode::accelNotchQValueCb(const long& p)
{
  acc_notch_q_ = p;
  return updateNotchParams();
}

bool ImuPreprocessNode::accelNotchDepthCb(const double& p)
{
  acc_notch_depth_ = p;
  return updateNotchParams();
}

bool ImuPreprocessNode::gyroLowPassCutoffCb(const long& p)
{
  if (!gyro_lpf_.setCutoffFrequency(p))
  {
    TOBAS_ERROR("Failed to set cutoff frequency of gyro low-pass filter.");
    return false;
  }

  return true;
}

bool ImuPreprocessNode::gyroNotchMinFreqCb(const long& p)
{
  gyro_notch_min_freq_ = p;
  return true;
}

bool ImuPreprocessNode::gyroNotchQValueCb(const long& p)
{
  gyro_notch_q_ = p;
  return updateNotchParams();
}

bool ImuPreprocessNode::gyroNotchDepthCb(const double& p)
{
  gyro_notch_depth_ = p;
  return updateNotchParams();
}

void ImuPreprocessNode::imuRawCb(const tobas_msgs::ImuStamped::ConstSharedPtr& imu_raw)
{
  switch (stage_)
  {
    case MEASURE_GYRO_BIAS:
    {
      // 角速度が大きすぎる場合はやり直し
      if (imu_raw->imu.gyro.norm() > kStaticGyroThreshold)
      {
        TOBAS_WARN("Perturbation is detected while measuring gyro bias: ", imu_raw->imu.gyro, " [rad/s]. Retrying...");
        gyro_bias_cnt_ = 0;
        for (size_t i = 0; i < 3; ++i)
          gyro_sum_[i].reset();
        break;
      }

      // 角速度を加算
      for (size_t i = 0; i < 3; ++i)
        gyro_sum_[i].add(imu_raw->imu.gyro(i));

      // データが溜まったら角速度の平均をバイアスの推定値として次のステージに進む
      if (++gyro_bias_cnt_ == kMeasureGyroBiasCount)
      {
        for (size_t i = 0; i < 3; ++i)
          gyro_bias_(i) = gyro_sum_[i].get() / kMeasureGyroBiasCount;
        TOBAS_INFO("Finished measuring gyro bias. It is estimated to be: ", gyro_bias_);
        stage_ = INITIALIZE;
      }

      break;
    }
    case INITIALIZE:
    {
      acc_lpf_.setValue(imu_raw->imu.accel);
      gyro_lpf_.setValue(imu_raw->imu.gyro);

      if (!acc_noise_.initialize(kHpfCutoff, imu_raw->imu.accel.data))
      {
        TOBAS_ERROR("Failed to initialize accel noise variance filter.");
        return;
      }
      if (!gyro_noise_.initialize(kHpfCutoff, imu_raw->imu.gyro.data))
      {
        TOBAS_ERROR("Failed to initialize gyro noise variance filter.");
        return;
      }

      imu_raw_ = imu_raw;
      stage_ = PUBLISH;
      break;
    }
    case PUBLISH:
    {
      // Compute time difference
      const auto dt = (imu_raw->header.stamp - imu_raw_->header.stamp).seconds();
      imu_raw_ = imu_raw;

      // Low-pass filter
      if (acc_lpf_.update(imu_raw->imu.accel, dt) < 0)
        TOBAS_ERROR("Failed to update accel Low-pass filter: ", acc_lpf_.errorMessage(), " dt = ", dt);
      if (gyro_lpf_.update(imu_raw->imu.gyro - gyro_bias_, dt) < 0)
        TOBAS_ERROR("Failed to update gyro Low-pass filter: ", gyro_lpf_.errorMessage(), " dt = ", dt);

      auto acc_out = acc_lpf_.getValue();
      auto gyro_out = gyro_lpf_.getValue();

      cout << "After Low-pass filter: " << acc_out << endl;

      // Notch filter
      const auto num_rotors = rot_speeds_.size();
      for (size_t harm_idx = 0; harm_idx < kNumNotchFilterPerRotor; ++harm_idx)
      {
        for (size_t rotor_idx = 0; rotor_idx < num_rotors; ++rotor_idx)
        {
          const auto notch_idx = harm_idx * num_rotors + rotor_idx;
          auto& acc_notch = acc_notch_.at(notch_idx);
          auto& gyro_notch = gyro_notch_.at(notch_idx);

          const auto rot_speed = rot_speeds_.at(rotor_idx);    // [rad/s]
          const auto harm_speed = rot_speed * (harm_idx + 1);  // [rad/s]
          const auto harm_freq = harm_speed / (2 * M_PI);      // [Hz]

          if (harm_freq >= acc_notch_min_freq_)
          {
            if (!acc_notch.setCenterFrequency(harm_freq))
              TOBAS_ERROR("Failed to set center frequency of accel notch filter to ", harm_freq, "[Hz].");
            if (acc_notch.update(acc_out, dt) < 0)
              TOBAS_ERROR("Failed to update accel notch filter: ", acc_notch.errorMessage());
          }
          else
          {
            acc_notch.bypass(acc_out);
          }

          if (harm_freq >= gyro_notch_min_freq_)
          {
            if (!gyro_notch.setCenterFrequency(harm_freq))
              TOBAS_ERROR("Failed to set center frequency of gyro notch filter to ", harm_freq, "[Hz].");
            if (gyro_notch.update(gyro_out, dt) < 0)
              TOBAS_ERROR("Failed to update gyro notch filter: ", gyro_notch.errorMessage());
          }
          else
          {
            gyro_notch.bypass(gyro_out);
          }

          acc_out = acc_notch.getValue();
          gyro_out = gyro_notch.getValue();

          cout << "After notch filter " << notch_idx << ": " << acc_out << endl;
        }
      }

      // Update noise filters
      if (acc_noise_.update(acc_out.data, dt) < 0)
        TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Accel noise filter failed: ", acc_noise_.errorMessage());
      if (gyro_noise_.update(gyro_out.data, dt) < 0)
        TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Gyro noise filter failed: ", gyro_noise_.errorMessage());

      // Create IMU message
      auto imu_out = std::make_unique<tobas_msgs::ImuWithCovarianceStamped>();
      imu_out->header = imu_raw->header;
      imu_out->imu.imu.accel = acc_out;
      imu_out->imu.imu.gyro = gyro_out;
      imu_out->imu.accel_covariance = acc_noise_.noiseVariance();
      imu_out->imu.gyro_covariance = gyro_noise_.noiseVariance();

      // Publish message
      imu_pub_->publish(move(imu_out));

      break;
    }
  }
}

void ImuPreprocessNode::rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& rotor_states)
{
  for (const auto& state : rotor_states->states)
  {
    if (state.status == tobas_msgs::msg::RotorState::NO_COMMUNICATION)
      continue;

    if (!channel2idx_.contains(state.channel))
    {
      TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Invalid rotor channel: ", state.channel);
      continue;
    }

    const auto& notch_idx = channel2idx_.at(state.channel);
    rot_speeds_.at(notch_idx) = state.speed;
  }
}

void ImuPreprocessNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  size_t idx = 0;
  channel2idx_.clear();
  for (const auto& [channel, rotor] : drone->rotors)
    channel2idx_[channel] = idx++;

  rot_speeds_.resize(drone->numRotors(), 0.);

  const auto num_notch = drone->numRotors() * kNumNotchFilterPerRotor;
  acc_notch_.resize(num_notch);
  gyro_notch_.resize(num_notch);

  updateNotchParams();
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuPreprocessNode)
