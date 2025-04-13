#include <boost/polymorphic_pointer_cast.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_path_tools/join.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/util.hpp>
#include <tobas_real_common/constants.hpp>

#include <tobas_kdl_msgs/msg/euler_stamped.hpp>
#include <tobas_kdl_msgs/msg/tree.hpp>
#include <tobas_std_msgs/msg/message.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/engine_state.hpp>
#include <tobas_msgs/msg/fluid_pressure_stamped.hpp>
#include <tobas_msgs/msg/gnss.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/imu_stamped.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/magnetic_field_stamped.hpp>
#include <tobas_msgs/msg/odometry.hpp>
#include <tobas_msgs/msg/sbus.hpp>
#include <tobas_msgs/msg/post_arm_check.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/rosbag_state.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
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
#include <tobas_real_msgs/srv/set_imu_params.hpp>
#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>
#include <tobas_real_msgs/srv/set_rc_input_params.hpp>

#define DEFAULT_NUM_THREADS 4
#define MIN_NUM_THREADS 2

class ROSInterfaceNode : public tobas::BaseNode
{
  using self = ROSInterfaceNode;
  using super = tobas::BaseNode;

public:
  explicit ROSInterfaceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<std::string, rclcpp::ServiceBase::SharedPtr> services_;
  std::map<std::string, rclcpp::ClientBase::SharedPtr> clients_;

  template <typename MsgType>
  void
  addTopic(const std::string& sub_topic, const std::string& pub_topic, bool latch, bool reliable, size_t queue_size);

  template <typename MsgType>
  void addTopicLogicToIface(
    const std::string& sub_topic,
    const std::string& pub_topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename MsgType>
  void addTopicIfaceToLogic(
    const std::string& sub_topic,
    const std::string& pub_topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename SrvType>
  void addService(const std::string& srv_name);

  template <typename MsgType>
  void topicCallback(const typename MsgType::ConstSharedPtr& msg_in, const std::string& pub_topic);

  template <typename SrvType>
  void serviceCallback(
    const typename SrvType::Request::SharedPtr& req,
    const typename SrvType::Response::SharedPtr& res,
    const std::string& srv_name);
};

ROSInterfaceNode::ROSInterfaceNode(const rclcpp::NodeOptions& options) : super("ros_interface", options)
{
  // サービスコールバックを再帰的に呼んだ際のデッドロックを回避
  // cf. https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  addTopicLogicToIface<tobas_std_msgs::msg::Message>(tobas::kMessageTopic, tobas::kMessageTopic);
  addTopicLogicToIface<tobas_drone_msgs::msg::Drone>(tobas::kDroneTopic, tobas::kDroneTopic, true, true);
  addTopicLogicToIface<tobas_kdl_msgs::msg::Tree>(tobas::kKdlTreeTopic, tobas::kKdlTreeTopic, true, true);
  addTopicLogicToIface<tobas_msgs::msg::Battery>(tobas::addThrotNS(tobas::kBatteryTopic), tobas::kBatteryTopic);
  addTopicLogicToIface<tobas_msgs::msg::EngineState>(
    tobas::addThrotNS(tobas::kEngineStateTopic), tobas::kEngineStateTopic);
  addTopicLogicToIface<tobas_msgs::msg::Cpu>(tobas::kCpuTopic, tobas::kCpuTopic);
  addTopicLogicToIface<tobas_msgs::msg::Sbus>(tobas::addThrotNS(tobas::kSbusTopic), tobas::kSbusTopic);
  addTopicLogicToIface<tobas_msgs::msg::RCInput>(tobas::addThrotNS(tobas::kRcInputTopic), tobas::kRcInputTopic);
  addTopicLogicToIface<tobas_msgs::msg::Gnss>(tobas::kGnssTopic, tobas::kGnssTopic);
  addTopicLogicToIface<tobas_msgs::msg::RotorStateArray>(
    tobas::addThrotNS(tobas::kRotorStatesTopic), tobas::kRotorStatesTopic);
  addTopicLogicToIface<tobas_msgs::msg::RotorLivelinessArray>(
    tobas::kRotorLivelinessTopic, tobas::kRotorLivelinessTopic);
  addTopicLogicToIface<tobas_msgs::msg::JointStateArray>(
    tobas::addThrotNS(tobas::kJointStatesTopic), tobas::kJointStatesTopic);
  addTopicLogicToIface<tobas_msgs::msg::Odometry>(tobas::addThrotNS(tobas::kOdometryTopic), tobas::kOdometryTopic);
  addTopicLogicToIface<tobas_msgs::msg::Arming>(tobas::kArmingTopic, tobas::kArmingTopic);
  addTopicLogicToIface<tobas_msgs::msg::PreArmCheck>(tobas::kPreArmCheckTopic, tobas::kPreArmCheckTopic);
  addTopicLogicToIface<tobas_msgs::msg::PostArmCheck>(tobas::kPostArmCheckTopic, tobas::kPostArmCheckTopic);
  addTopicLogicToIface<tobas_msgs::msg::ImuStamped>(tobas::addThrotNS(real::kImuTopic), real::kImuTopic);
  addTopicLogicToIface<tobas_msgs::msg::MagneticFieldStamped>(tobas::addThrotNS(real::kMagTopic), real::kMagTopic);
  addTopicLogicToIface<tobas_msgs::msg::RosbagState>(tobas::kRosbagStateTopic, tobas::kRosbagStateTopic);

  addTopicIfaceToLogic<tobas_msgs::msg::RotorSpeedArray>(tobas::kRotorSpeedsCmdTopic, tobas::kRotorSpeedsCmdTopic);
  addTopicIfaceToLogic<tobas_msgs::msg::IcePropulsionSystemCommand>(
    tobas::kIcePropulsionSystemCmdTopic, tobas::kIcePropulsionSystemCmdTopic);
  addTopicIfaceToLogic<tobas_msgs::msg::JointCommandArray>(tobas::kJointPosCmdTopic, tobas::kJointPosCmdTopic);
  addTopicIfaceToLogic<tobas_msgs::msg::JointCommandArray>(tobas::kJointVelCmdTopic, tobas::kJointVelCmdTopic);
  addTopicIfaceToLogic<tobas_msgs::msg::JointCommandArray>(tobas::kJointEffCmdTopic, tobas::kJointEffCmdTopic);

  addService<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv);
  addService<tobas_msgs::srv::GetGnssOrigin>(tobas::kGetGnssOriginSrv);
  addService<tobas_msgs::srv::SetGnssOrigin>(tobas::kSetGnssOriginSrv);
  addService<tobas_msgs::srv::BagRecordStart>(tobas::kRosbagRecordStartSrv);
  addService<tobas_msgs::srv::BagRecordStop>(tobas::kRosbagRecordStopSrv);
  addService<std_srvs::srv::Trigger>(tobas::kRosbagCleanSrv);
  addService<tobas_msgs::srv::GetRotorControlGains>(tobas::kGetRotorControlGainsSrv);
  addService<tobas_msgs::srv::SetRotorControlGains>(tobas::kSetRotorControlGainsSrv);
  addService<std_srvs::srv::Trigger>(tobas::kSaveRotorControlGainsSrv);
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(tobas::node::kController, tobas::kGetDynamicParamsSrv));
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(tobas::node::kObserver, tobas::kGetDynamicParamsSrv));
  addService<tobas_real_msgs::srv::SetImuParams>(real::handler::imu::kSetParamSrv);
  addService<tobas_real_msgs::srv::SetMagnetometerParams>(real::handler::mag::kSetParamSrv);
  addService<tobas_real_msgs::srv::SetRcInputParams>(real::handler::rcin::kSetParamSrv);
}

template <typename MsgType>
void ROSInterfaceNode::addTopic(
  const std::string& sub_topic,
  const std::string& pub_topic,
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
  const std::string& sub_topic,
  const std::string& pub_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  addTopic<MsgType>(sub_topic, tobas::addIfaceNS(pub_topic), latch, reliable, queue_size);
}

template <typename MsgType>
void ROSInterfaceNode::addTopicIfaceToLogic(
  const std::string& sub_topic,
  const std::string& pub_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  addTopic<MsgType>(tobas::addIfaceNS(sub_topic), pub_topic, latch, reliable, queue_size);
}

template <typename SrvType>
void ROSInterfaceNode::addService(const std::string& srv_name)
{
  auto cb =
    [this, srv_name](const typename SrvType::Request::SharedPtr& req, const typename SrvType::Response::SharedPtr& res)
  { serviceCallback<SrvType>(req, res, srv_name); };
  services_[srv_name] =
    create_service<SrvType>(tobas::addIfaceNS(srv_name), cb, rclcpp::ServicesQoS(), callback_group_);

  clients_[srv_name] = create_client<SrvType>(srv_name);
}

template <typename MsgType>
void ROSInterfaceNode::topicCallback(const typename MsgType::ConstSharedPtr& msg_in, const std::string& pub_topic)
{
  auto msg_out = std::make_unique<MsgType>(*msg_in);
  const auto publisher = boost::polymorphic_pointer_downcast<rclcpp::Publisher<MsgType>>(publishers_.at(pub_topic));
  publisher->publish(move(msg_out));
}

template <typename SrvType>
void ROSInterfaceNode::serviceCallback(
  const typename SrvType::Request::SharedPtr& req,
  const typename SrvType::Response::SharedPtr& res,
  const std::string& srv_name)
{
  const auto client = boost::polymorphic_pointer_downcast<rclcpp::Client<SrvType>>(clients_.at(srv_name));

  if (!client->wait_for_service())
  {
    TOBAS_ERROR("\"", client->get_service_name(), "\" service is not ready.");
    return;
  }

  auto future = client->async_send_request(req);
  future.wait();

  *res = *future.get();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<ROSInterfaceNode>();

  long num_threads = DEFAULT_NUM_THREADS;
  if (node->has_parameter("num_threads"))
    num_threads = node->get_parameter("num_threads").as_int();

  if (num_threads < MIN_NUM_THREADS)
  {
    RCLCPP_WARN_STREAM(
      node->get_logger(),
      "To avoid deadlock with recursive service calls, at least " << MIN_NUM_THREADS << " threads are required.");
    num_threads = MIN_NUM_THREADS;
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "The number of threads is set to " << num_threads << ".");

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), num_threads);
  exec.add_node(node);

  exec.spin();
}
