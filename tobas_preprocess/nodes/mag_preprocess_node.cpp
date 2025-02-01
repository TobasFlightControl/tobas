#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/magnetic_field_stamped.hpp>
#include <tobas_msgs_adapter/magnetic_field_with_covariance_stamped.hpp>

class MagPreprocessNode : public tobas::BaseNode
{
  using self = MagPreprocessNode;
  using super = tobas::BaseNode;

  static constexpr double kMagScale = 5e+4;  // TODO: 経緯高度による変化を反映

public:
  explicit MagPreprocessNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  Eigen::Matrix3d mag_noise_cov_;  // [-]

  ros2::PublisherPtr<tobas_msgs::MagneticFieldWithCovarianceStamped> mag_pub_;
  ros2::SubscriberPtr<tobas_msgs::MagneticFieldStamped> mag_raw_sub_;

  bool magNoiseStddevCb(const long& p);

  void magRawCb(const tobas_msgs::MagneticFieldStamped::ConstSharedPtr& mag_raw);
};

MagPreprocessNode::MagPreprocessNode(const rclcpp::NodeOptions& options) : super("mag_preprocess", options)
{
  // 地磁気はサンプリング周波数が小さくオンライン分散推定は困難なため，固定値を用いる．
  // IIS2MDCのデータシートによると，LPF付きでノイズのRMS (= 標準偏差) の最大値が4.6mG (= 460nT)．
  // ナイキスト周波数を考慮したLPFを通さない場合はその√2倍で650nTほどだと思われる．
  // 電源やモータ等の環境ノイズも考えて，デフォルト値はそれよりさらに大きい値に設定．
  addDynamicIntParam("mag_noise_stddev", &self::magNoiseStddevCb, this, 1000, 1, 5000);  // [nT]

  mag_pub_ = createPublisher<tobas_msgs::MagneticFieldWithCovarianceStamped>(tobas::kMagTopic);
  mag_raw_sub_ = createSubscriber(tobas::kMagRawTopic, &self::magRawCb, this);
}

bool MagPreprocessNode::magNoiseStddevCb(const long& p)
{
  const auto noise_stddev = static_cast<double>(p) / kMagScale;  // [-]
  const auto noise_var = math::sqr(noise_stddev);
  mag_noise_cov_ = Eigen::Vector3d::Constant(noise_var).asDiagonal();
  return true;
}

void MagPreprocessNode::magRawCb(const tobas_msgs::MagneticFieldStamped::ConstSharedPtr& mag_raw)
{
  // Create message
  auto mag_out = std::make_unique<tobas_msgs::MagneticFieldWithCovarianceStamped>();
  mag_out->header = mag_raw->header;
  mag_out->mag.mag = mag_raw->mag;
  mag_out->mag.covariance = mag_noise_cov_;

  // Publish message
  mag_pub_->publish(std::move(mag_out));
}

RCLCPP_COMPONENTS_REGISTER_NODE(MagPreprocessNode)
