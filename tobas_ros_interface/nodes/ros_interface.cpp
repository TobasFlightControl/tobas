#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_path_tools/join.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_real_common/constants.hpp>

#include <tobas_kdl_msgs/msg/euler_stamped.hpp>
#include <tobas_kdl_msgs/msg/tree.hpp>
#include <tobas_std_msgs/msg/message.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/gps.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs/srv/get_gnss_origin.hpp>
#include <tobas_msgs/srv/set_gnss_origin.hpp>
#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>
#include <tobas_msgs/srv/get_rotor_control_gains.hpp>
#include <tobas_msgs/srv/set_rotor_control_gains.hpp>
#include <tobas_drone_msgs/msg/drone.hpp>
#include <tobas_dparam_msgs/srv/get_params.hpp>
#include <tobas_msgs/msg/imu_stamped.hpp>
#include <tobas_msgs/msg/magnetic_field_stamped.hpp>
#include <tobas_msgs/msg/fluid_pressure_stamped.hpp>
#include <tobas_msgs/msg/sbus.hpp>
#include <tobas_real_msgs/srv/set_imu_params.hpp>
#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>
#include <tobas_real_msgs/srv/set_battery_params.hpp>
#include <tobas_real_msgs/srv/set_rc_input_params.hpp>

using namespace std;

class ROSInterfaceNode : public tobas::BaseNode
{
  using self = ROSInterfaceNode;
  using super = tobas::BaseNode;

public:
  explicit ROSInterfaceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  map<string, rclcpp::PublisherBase::SharedPtr> publishers_;
  map<string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  map<string, rclcpp::ServiceBase::SharedPtr> services_;
  map<string, rclcpp::ClientBase::SharedPtr> clients_;

  template <typename MsgType>
  void addTopic(const string& sub_topic, const string& pub_topic, bool latch, bool reliable, size_t queue_size);

  template <typename MsgType>
  void addTopicLogicToIface(
    const string& sub_topic,
    const string& pub_topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename MsgType>
  void addTopicIfaceToLogic(
    const string& sub_topic,
    const string& pub_topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename SrvType>
  void addService(const string& srv_name);

  template <typename MsgType>
  void topicCallback(const typename MsgType::ConstSharedPtr& msg_in, const string& pub_topic);

  template <typename SrvType>
  void serviceCallback(
    const typename SrvType::Request::SharedPtr& req,
    const typename SrvType::Response::SharedPtr& res,
    const string& srv_name);

  static string throttled(const string& topic);
  static string interface(const string& name);
};

ROSInterfaceNode::ROSInterfaceNode(const rclcpp::NodeOptions& options) : super("ros_interface", options)
{
  // サービスコールバックを再帰的に呼んだ際のデッドロックを回避
  // cf. https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  addTopicLogicToIface<tobas_std_msgs::msg::Message>(tobas::kMessageTopic, tobas::kMessageTopic);
  addTopicLogicToIface<tobas_drone_msgs::msg::Drone>(tobas::kDroneTopic, tobas::kDroneTopic, true, true);
  addTopicLogicToIface<tobas_kdl_msgs::msg::Tree>(tobas::kKDLTreeTopic, tobas::kKDLTreeTopic, true, true);
  addTopicLogicToIface<tobas_msgs::msg::Battery>(throttled(tobas::kBatteryTopic), tobas::kBatteryTopic);
  addTopicLogicToIface<tobas_msgs::msg::Cpu>(tobas::kCPUTopic, tobas::kCPUTopic);
  addTopicLogicToIface<tobas_msgs::msg::RCInput>(throttled(tobas::kRcInputTopic), tobas::kRcInputTopic);
  addTopicLogicToIface<tobas_msgs::msg::Gps>(tobas::kGNSSTopic, tobas::kGNSSTopic);
  addTopicLogicToIface<tobas_msgs::msg::RotorStateArray>(throttled(tobas::kRotorStatesTopic), tobas::kRotorStatesTopic);
  addTopicLogicToIface<tobas_kdl_msgs::msg::EulerStamped>(throttled(tobas::kEulerTopic), tobas::kEulerTopic);
  addTopicLogicToIface<std_msgs::msg::Bool>(tobas::kArmingTopic, tobas::kArmingTopic);
  addTopicLogicToIface<tobas_msgs::msg::PreArmCheck>(tobas::kPreArmCheckTopic, tobas::kPreArmCheckTopic);
  addTopicLogicToIface<tobas_msgs::msg::ImuStamped>(throttled(real::kIMUTopic), real::kIMUTopic);
  addTopicLogicToIface<tobas_msgs::msg::MagneticFieldStamped>(throttled(real::kMagTopic), real::kMagTopic);
  addTopicLogicToIface<tobas_msgs::msg::Sbus>(throttled(real::kSBUSTopic), real::kSBUSTopic);

  addTopicIfaceToLogic<tobas_msgs::msg::RotorSpeedArray>(tobas::kRotorSpeedsCmdTopic, tobas::kRotorSpeedsCmdTopic);

  addService<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv);
  addService<tobas_msgs::srv::GetGnssOrigin>(tobas::kGetGnssOriginSrv);
  addService<tobas_msgs::srv::SetGnssOrigin>(tobas::kSetGnssOriginSrv);
  addService<tobas_msgs::srv::BagRecordStart>(tobas::kROSBagRecordStartSrv);
  addService<tobas_msgs::srv::BagRecordStop>(tobas::kROSBagRecordStopSrv);
  addService<tobas_msgs::srv::GetRotorControlGains>(tobas::kGetRotorControlGainsSrv);
  addService<tobas_msgs::srv::SetRotorControlGains>(tobas::kSetRotorControlGainsSrv);
  addService<std_srvs::srv::Trigger>(tobas::kSaveRotorControlGainsSrv);
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(tobas::kControllerNode, tobas::kGetDynamicParamsSrv));
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(tobas::kObserverNode, tobas::kGetDynamicParamsSrv));
  addService<tobas_real_msgs::srv::SetIMUParams>(real::handler::imu::kSetParamSrv);
  addService<tobas_real_msgs::srv::SetMagnetometerParams>(real::handler::mag::kSetParamSrv);
  addService<tobas_real_msgs::srv::SetBatteryParams>(real::handler::adc::kSetParamSrv);
  addService<tobas_real_msgs::srv::SetRCInputParams>(real::handler::rcin::kSetParamSrv);
}

template <typename MsgType>
void ROSInterfaceNode::addTopic(
  const string& sub_topic,
  const string& pub_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  const auto qos = ros2::makeQoS(latch, reliable, queue_size);

  const auto cb = [this, pub_topic](const typename MsgType::ConstSharedPtr& msg)
  { topicCallback<MsgType>(msg, pub_topic); };
  subscriptions_[sub_topic] = create_subscription<MsgType>(sub_topic, qos, cb);

  publishers_[pub_topic] = create_publisher<MsgType>(pub_topic, qos);
}

template <typename MsgType>
void ROSInterfaceNode::addTopicLogicToIface(
  const string& sub_topic,
  const string& pub_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  addTopic<MsgType>(sub_topic, interface(pub_topic), latch, reliable, queue_size);
}

template <typename MsgType>
void ROSInterfaceNode::addTopicIfaceToLogic(
  const string& sub_topic,
  const string& pub_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  addTopic<MsgType>(interface(sub_topic), pub_topic, latch, reliable, queue_size);
}

template <typename SrvType>
void ROSInterfaceNode::addService(const string& srv_name)
{
  auto cb =
    [this, srv_name](const typename SrvType::Request::SharedPtr& req, const typename SrvType::Response::SharedPtr& res)
  { serviceCallback<SrvType>(req, res, srv_name); };
  services_[srv_name] = create_service<SrvType>(interface(srv_name), cb, rclcpp::ServicesQoS(), callback_group_);

  clients_[srv_name] = create_client<SrvType>(srv_name);
}

template <typename MsgType>
void ROSInterfaceNode::topicCallback(const typename MsgType::ConstSharedPtr& msg_in, const string& pub_topic)
{
  auto msg_out = std::make_unique<MsgType>(*msg_in);
  const auto publisher = dynamic_pointer_cast<rclcpp::Publisher<MsgType>>(publishers_.at(pub_topic));
  publisher->publish(move(msg_out));
}

template <typename SrvType>
void ROSInterfaceNode::serviceCallback(
  const typename SrvType::Request::SharedPtr& req,
  const typename SrvType::Response::SharedPtr& res,
  const string& srv_name)
{
  const auto client = dynamic_pointer_cast<rclcpp::Client<SrvType>>(clients_.at(srv_name));

  if (!client->wait_for_service())
  {
    TOBAS_ERROR("\"", client->get_service_name(), "\" service is not ready.");
    return;
  }

  auto future = client->async_send_request(req);
  future.wait();

  *res = *future.get();
}

string ROSInterfaceNode::throttled(const string& topic)
{
  return path::join(tobas::kThrottledTopicNS, topic);
}

string ROSInterfaceNode::interface(const string& name)
{
  return path::join(tobas::kRemoteIfaceTopicNS, name);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const auto node = make_shared<ROSInterfaceNode>();

  long num_threads = 4;
  if (node->has_parameter("num_threads"))
    num_threads = node->get_parameter("num_threads").as_int();

  if (num_threads == 1)
  {
    RCLCPP_WARN(node->get_logger(), "To avoid deadlock with recursive service calls, at least 2 threads are required.");
    num_threads = 2;
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "The number of threads is set to " << num_threads << ".");

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), num_threads);
  exec.add_node(node);

  exec.spin();
}
