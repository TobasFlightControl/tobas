#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_kdl_msgs/EulerStamped.hpp>
#include <tobas_msgs/Odometry.hpp>

using namespace std;

/**
 * @brief オドメトリから得られた姿勢をオイラー角に変換して発行する．
 */
class MatrixEulerConverterNode : public tobas::BaseNode
{
  using self = MatrixEulerConverterNode;
  using super = tobas::BaseNode;

public:
  explicit MatrixEulerConverterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<tobas_kdl_msgs::EulerStamped> euler_pub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
};

MatrixEulerConverterNode::MatrixEulerConverterNode(const rclcpp::NodeOptions& options)
  : super("matrix_euler_converter", options)
{
  euler_pub_ = createPublisher<tobas_kdl_msgs::EulerStamped>(tobas::kEulerTopic);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
}

void MatrixEulerConverterNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  auto euler = std::make_unique<tobas_kdl_msgs::EulerStamped>();
  euler->header = odom->header;
  odom->frame.M.getRPY(euler->euler.roll, euler->euler.pitch, euler->euler.yaw);
  euler_pub_->publish(move(euler));
}

RCLCPP_COMPONENTS_REGISTER_NODE(MatrixEulerConverterNode)
