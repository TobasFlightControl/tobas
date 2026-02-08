#include <tobas_constants/constants.hpp>
#include <tobas_node/node.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_tools/util.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <tobas_command_msgs/msg/accel.hpp>
#include <tobas_command_msgs/msg/accel_pitch_yaw.hpp>
#include <tobas_command_msgs/msg/accel_yaw.hpp>
#include <tobas_command_msgs/msg/angle.hpp>
#include <tobas_command_msgs/msg/angle_throttle.hpp>
#include <tobas_command_msgs/msg/pos_vel.hpp>
#include <tobas_command_msgs/msg/pos_vel_pitch_yaw.hpp>
#include <tobas_command_msgs/msg/pos_vel_yaw.hpp>
#include <tobas_command_msgs/msg/rate.hpp>
#include <tobas_command_msgs/msg/rate_throttle.hpp>
#include <tobas_command_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_dparam_msgs/srv/get_params.hpp>
#include <tobas_drone_msgs/msg/drone.hpp>
#include <tobas_kdl_msgs/msg/euler_stamped.hpp>
#include <tobas_kdl_msgs/msg/tree.hpp>
#include <tobas_mission_msgs/action/execute_mission.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/engine_state.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/gnss.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/imu.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/magnetic_field.hpp>
#include <tobas_msgs/msg/message.hpp>
#include <tobas_msgs/msg/odometry.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/rosbag_state.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/sbus.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>
#include <tobas_msgs/srv/get_gnss_origin.hpp>
#include <tobas_msgs/srv/get_rotor_control_gains.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs/srv/set_gnss_origin.hpp>
#include <tobas_msgs/srv/set_rotor_control_gains.hpp>
#include <tobas_real_msgs/srv/set_imu_params.hpp>
#include <tobas_real_msgs/srv/set_magnetometer_params.hpp>
#include <tobas_real_msgs/srv/set_rc_input_params.hpp>

struct TopicBase
{
  using SharedPtr = std::shared_ptr<TopicBase>;
};

template <typename MsgType>
struct Topic : public TopicBase
{
  using SharedPtr = std::shared_ptr<Topic<MsgType>>;

  rclcpp::Publisher<MsgType>::SharedPtr publisher;
  rclcpp::Subscription<MsgType>::SharedPtr subscriber;
};

struct ServiceBase
{
  using SharedPtr = std::shared_ptr<ServiceBase>;

  std::string name;
};

template <typename SrvType>
struct Service : public ServiceBase
{
  using SharedPtr = std::shared_ptr<Service<SrvType>>;

  rclcpp::Service<SrvType>::SharedPtr server;
  rclcpp::Client<SrvType>::SharedPtr client;
};

struct ActionBase
{
  using SharedPtr = std::shared_ptr<ActionBase>;

  std::string name;
};

template <typename ActType>
struct Action : public ActionBase
{
  using SharedPtr = std::shared_ptr<Action<ActType>>;

  rclcpp_action::Server<ActType>::SharedPtr server;
  rclcpp_action::Client<ActType>::SharedPtr client;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>> server_gh;
  rclcpp_action::ClientGoalHandle<ActType>::SharedPtr client_gh;
};

class RosInterfaceNode : public tobas::BaseNode
{
  using self = RosInterfaceNode;
  using super = tobas::BaseNode;

public:
  explicit RosInterfaceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::CallbackGroup::SharedPtr group_;

  std::vector<TopicBase::SharedPtr> topics_;
  std::vector<ServiceBase::SharedPtr> services_;
  std::vector<ActionBase::SharedPtr> actions_;

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
  void addService(const std::string& service_name);

  template <typename ActType>
  void addAction(const std::string& action_name);

  template <typename MsgType>
  void topicCallback(Topic<MsgType>::SharedPtr topic, const typename MsgType::ConstSharedPtr& msg);

  template <typename SrvType>
  void serviceCallback(
    Service<SrvType>::SharedPtr service,
    const typename SrvType::Request::SharedPtr& req,
    const typename SrvType::Response::SharedPtr& res);

  template <typename ActType>
  void actionFeedbackCallback(Action<ActType>::SharedPtr action, const typename ActType::Feedback::ConstSharedPtr& fb);

  template <typename ActType>
  rclcpp_action::GoalResponse actionHandleGoal(
    Action<ActType>::SharedPtr action,
    const rclcpp_action::GoalUUID& uuid,
    const typename ActType::Goal::ConstSharedPtr& goal);

  template <typename ActType>
  rclcpp_action::CancelResponse actionHandleCancel(
    Action<ActType>::SharedPtr action,
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& gh);

  template <typename ActType>
  void actionHandleAccepted(
    Action<ActType>::SharedPtr action,
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& gh);
};

RosInterfaceNode::RosInterfaceNode(const rclcpp::NodeOptions& options) : super("ros_interface", options)
{
  // サービスコールバックを再帰的に呼んだ際のデッドロックを回避
  // cf. https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/
  group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  addTopicLogicToIface<tobas_msgs::msg::Message>(tobas::kMessageTopic, tobas::kMessageTopic);
  addTopicLogicToIface<tobas_drone_msgs::msg::Drone>(tobas::kDroneTopic, tobas::kDroneTopic, true, true);
  addTopicLogicToIface<tobas_kdl_msgs::msg::Tree>(tobas::kKdlTreeTopic, tobas::kKdlTreeTopic, true, true);
  addTopicLogicToIface<tobas_msgs::msg::Battery>(tobas::addThrotNS(tobas::kBatteryTopic), tobas::kBatteryTopic);
  addTopicLogicToIface<tobas_msgs::msg::EngineState>(
    tobas::addThrotNS(tobas::kEngineStateTopic), tobas::kEngineStateTopic);
  addTopicLogicToIface<tobas_msgs::msg::Cpu>(tobas::kCpuTopic, tobas::kCpuTopic);
  addTopicLogicToIface<tobas_msgs::msg::Sbus>(tobas::addThrotNS(tobas::kSbusTopic), tobas::kSbusTopic);
  addTopicLogicToIface<tobas_msgs::msg::RCInput>(tobas::addThrotNS(tobas::kRcInputTopic), tobas::kRcInputTopic);
  addTopicLogicToIface<tobas_msgs::msg::Imu>(tobas::addThrotNS(tobas::kImuFiltTopic), tobas::kImuFiltTopic);
  addTopicLogicToIface<tobas_msgs::msg::MagneticField>(tobas::addThrotNS(tobas::kMagTopic), tobas::kMagTopic);
  addTopicLogicToIface<tobas_msgs::msg::FluidPressure>(
    tobas::addThrotNS(tobas::kAirPressureTopic), tobas::kAirPressureTopic);
  addTopicLogicToIface<tobas_msgs::msg::Gnss>(tobas::kGnssTopic, tobas::kGnssTopic);
  addTopicLogicToIface<tobas_msgs::msg::RotorStateArray>(
    tobas::addThrotNS(tobas::kRotorStatesTopic), tobas::kRotorStatesTopic);
  addTopicLogicToIface<tobas_msgs::msg::RotorLivelinessArray>(tobas::kRotorLivTopic, tobas::kRotorLivTopic);
  addTopicLogicToIface<tobas_msgs::msg::JointStateArray>(
    tobas::addThrotNS(tobas::kJointStatesTopic), tobas::kJointStatesTopic);
  addTopicLogicToIface<tobas_msgs::msg::Odometry>(tobas::addThrotNS(tobas::kOdometryTopic), tobas::kOdometryTopic);
  addTopicLogicToIface<tobas_msgs::msg::Arming>(tobas::kArmingTopic, tobas::kArmingTopic);
  addTopicLogicToIface<tobas_msgs::msg::VehicleHealth>(tobas::kVehicleHealthTopic, tobas::kVehicleHealthTopic);
  addTopicLogicToIface<tobas_msgs::msg::Imu>(tobas::addThrotNS(real::kImuRawTopic), real::kImuRawTopic);
  addTopicLogicToIface<tobas_msgs::msg::MagneticField>(tobas::addThrotNS(real::kMagTopic), real::kMagTopic);
  addTopicLogicToIface<tobas_msgs::msg::RosbagState>(tobas::kRosbagStateTopic, tobas::kRosbagStateTopic);

  // Low Command
  addTopicIfaceToLogic<tobas_msgs::msg::RotorSpeedArray>(tobas::kRotorSpeedsCmdTopic, tobas::kRotorSpeedsCmdTopic);
  addTopicIfaceToLogic<tobas_msgs::msg::IcePropulsionSystemCommand>(
    tobas::kIcePropulsionSystemCmdTopic, tobas::kIcePropulsionSystemCmdTopic);

  // High Command
  addTopicIfaceToLogic<tobas_command_msgs::msg::Rate>(tobas::kRateCmdTopic, tobas::kRateCmdTopic);
  addTopicIfaceToLogic<tobas_command_msgs::msg::RateThrottle>(tobas::kRateThrotCmdTopic, tobas::kRateThrotCmdTopic);
  addTopicIfaceToLogic<tobas_command_msgs::msg::Angle>(tobas::kAngleCmdTopic, tobas::kAngleCmdTopic);
  addTopicIfaceToLogic<tobas_command_msgs::msg::AngleThrottle>(tobas::kAngleThrotCmdTopic, tobas::kAngleThrotCmdTopic);
  addTopicIfaceToLogic<tobas_command_msgs::msg::Accel>(tobas::kAccelCmdTopic, tobas::kAccelCmdTopic);
  addTopicIfaceToLogic<tobas_command_msgs::msg::AccelYaw>(tobas::kAccelYawCmdTopic, tobas::kAccelYawCmdTopic);
  addTopicIfaceToLogic<tobas_command_msgs::msg::AccelPitchYaw>(
    tobas::kAccelPitchYawCmdTopic, tobas::kAccelPitchYawCmdTopic);
  addTopicIfaceToLogic<tobas_command_msgs::msg::PosVel>(tobas::kPosVelCmdTopic, tobas::kPosVelCmdTopic);
  addTopicIfaceToLogic<tobas_command_msgs::msg::PosVelYaw>(tobas::kPosVelYawCmdTopic, tobas::kPosVelYawCmdTopic);
  addTopicIfaceToLogic<tobas_command_msgs::msg::PosVelPitchYaw>(
    tobas::kPosVelPitchYawCmdTopic, tobas::kPosVelPitchYawCmdTopic);
  addTopicIfaceToLogic<tobas_command_msgs::msg::SpeedRollDeltaPitch>(
    tobas::kSpeedRollDpitchCmdTopic, tobas::kSpeedRollDpitchCmdTopic);

  // Joint Command
  addTopicIfaceToLogic<tobas_msgs::msg::JointCommandArray>(tobas::kJointPosCmdTopic, tobas::kJointPosCmdTopic);
  addTopicIfaceToLogic<tobas_msgs::msg::JointCommandArray>(tobas::kJointVelCmdTopic, tobas::kJointVelCmdTopic);
  addTopicIfaceToLogic<tobas_msgs::msg::JointCommandArray>(tobas::kJointEffCmdTopic, tobas::kJointEffCmdTopic);

  // Services
  addService<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv);
  addService<tobas_msgs::srv::GetGnssOrigin>(tobas::kGetGnssOriginSrv);
  addService<tobas_msgs::srv::SetGnssOrigin>(tobas::kSetGnssOriginSrv);
  addService<tobas_msgs::srv::BagRecordStart>(tobas::kRosbagRecordStartSrv);
  addService<tobas_msgs::srv::BagRecordStop>(tobas::kRosbagRecordStopSrv);
  addService<std_srvs::srv::Trigger>(tobas::kRosbagCleanSrv);
  addService<tobas_msgs::srv::GetRotorControlGains>(tobas::kGetRotorControlGainsSrv);
  addService<tobas_msgs::srv::SetRotorControlGains>(tobas::kSetRotorControlGainsSrv);
  addService<std_srvs::srv::Trigger>(tobas::kSaveRotorControlGainsSrv);
  addService<tobas_dparam_msgs::srv::GetParams>(
    path::join(tobas::node::kImuFilterConfigServer, tobas::kGetDynamicParamsSrv));
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(tobas::node::kObserver, tobas::kGetDynamicParamsSrv));
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(tobas::node::kController, tobas::kGetDynamicParamsSrv));
  addService<tobas_dparam_msgs::srv::GetParams>(path::join(tobas::node::kRcTeleop, tobas::kGetDynamicParamsSrv));
  addService<tobas_real_msgs::srv::SetImuParams>(real::handler::imu::kSetParamSrv);
  addService<tobas_real_msgs::srv::SetMagnetometerParams>(real::handler::mag::kSetParamSrv);
  addService<tobas_real_msgs::srv::SetRcInputParams>(real::handler::rcin::kSetParamSrv);

  // Actions
  addAction<tobas_mission_msgs::action::ExecuteMission>(tobas::kExecuteMissionAction);
}

template <typename MsgType>
void RosInterfaceNode::addTopic(
  const std::string& sub_topic,
  const std::string& pub_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  const auto topic = std::make_shared<Topic<MsgType>>();

  const auto qos = ros2::makeQoS(latch, reliable, queue_size);

  // Create publisher
  topic->publisher = create_publisher<MsgType>(pub_topic, qos);

  // Create subscriber
  const auto cb = [this, topic](const typename MsgType::ConstSharedPtr& msg) { topicCallback<MsgType>(topic, msg); };
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = group_;
  topic->subscriber = create_subscription<MsgType>(sub_topic, qos, cb, opts);

  topics_.push_back(topic);
}

template <typename MsgType>
void RosInterfaceNode::addTopicLogicToIface(
  const std::string& sub_topic,
  const std::string& pub_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  addTopic<MsgType>(sub_topic, tobas::addIfaceNS(pub_topic), latch, reliable, queue_size);
}

template <typename MsgType>
void RosInterfaceNode::addTopicIfaceToLogic(
  const std::string& sub_topic,
  const std::string& pub_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  addTopic<MsgType>(tobas::addIfaceNS(sub_topic), pub_topic, latch, reliable, queue_size);
}

template <typename SrvType>
void RosInterfaceNode::addService(const std::string& service_name)
{
  const auto service = std::make_shared<Service<SrvType>>();
  service->name = service_name;

  // Create server
  const auto cb =
    [this, service](const typename SrvType::Request::SharedPtr& req, const typename SrvType::Response::SharedPtr& res)
  { serviceCallback<SrvType>(service, req, res); };
  const auto qos = rclcpp::ServicesQoS();
  service->server = create_service<SrvType>(tobas::addIfaceNS(service_name), cb, qos, group_);

  // Create client
  service->client = create_client<SrvType>(service_name);

  services_.push_back(service);
}

template <typename ActType>
void RosInterfaceNode::addAction(const std::string& action_name)
{
  using GoalPtr = typename ActType::Goal::ConstSharedPtr;
  using GoalHandlePtr = std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>;

  const auto action = std::make_shared<Action<ActType>>();
  action->name = action_name;

  // Create server
  const auto handle_goal = [this, action](const rclcpp_action::GoalUUID& uuid, const GoalPtr& goal)
  { return actionHandleGoal<ActType>(action, uuid, goal); };
  const auto handle_cancel = [this, action](const GoalHandlePtr& gh)
  { return actionHandleCancel<ActType>(action, gh); };
  const auto handle_accepted = [this, action](const GoalHandlePtr& gh) { actionHandleAccepted<ActType>(action, gh); };
  action->server = rclcpp_action::create_server<ActType>(
    this,
    tobas::addIfaceNS(action_name),
    handle_goal,
    handle_cancel,
    handle_accepted,
    rcl_action_server_get_default_options(),
    group_);

  // Create client
  action->client = rclcpp_action::create_client<ActType>(this, action_name, group_);

  actions_.push_back(action);
}

template <typename MsgType>
void RosInterfaceNode::topicCallback(Topic<MsgType>::SharedPtr topic, const MsgType::ConstSharedPtr& msg)
{
  // 購読者はプロセス外のみの想定なので unique_ptr は作らずコピー無しで発行する
  topic->publisher->publish(*msg);
}

template <typename SrvType>
void RosInterfaceNode::serviceCallback(
  Service<SrvType>::SharedPtr service,
  const typename SrvType::Request::SharedPtr& req,
  const typename SrvType::Response::SharedPtr& res)
{
  if (!service->client->service_is_ready()) {
    TOBAS_ERROR("\"", service->name, "\" service is not ready.");
    return;
  }

  auto future = service->client->async_send_request(req);  // req が ConstSharedPtr だとここでコケる
  future.wait();

  *res = *future.get();  // future が const だとここでコケる
}

template <typename ActType>
void RosInterfaceNode::actionFeedbackCallback(
  Action<ActType>::SharedPtr action,
  const typename ActType::Feedback::ConstSharedPtr& fb_in)
{
  if (!action->server_gh) {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Waiting for \"", action->name, "\" action server goal handle.");
    return;
  }

  const auto fb_out = std::make_shared<typename ActType::Feedback>(*fb_in);  // const を外す
  action->server_gh->publish_feedback(fb_out);
}

template <typename ActType>
rclcpp_action::GoalResponse RosInterfaceNode::actionHandleGoal(
  Action<ActType>::SharedPtr action,
  const rclcpp_action::GoalUUID&,
  const typename ActType::Goal::ConstSharedPtr& goal)
{
  using Client = rclcpp_action::Client<ActType>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActType>;

  if (!action->client->action_server_is_ready()) {
    TOBAS_WARN("\"", action->name, "\" action is not ready.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // アクションフィードバックのコールバックを指定
  typename Client::SendGoalOptions opts;
  opts.feedback_callback =
    [this, action](const GoalHandle::SharedPtr&, const typename ActType::Feedback::ConstSharedPtr& fb)
  { actionFeedbackCallback<ActType>(action, fb); };

  // アクションゴールの確認が終了するまで待機
  const auto future = action->client->async_send_goal(*goal, opts);
  future.wait();

  // アクションが実行可能かどうかを確認
  action->client_gh = future.get();
  if (action->client_gh) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // -> actionHandleAccepted が実行される
  }
  else {
    return rclcpp_action::GoalResponse::REJECT;
  }
}

template <typename ActType>
rclcpp_action::CancelResponse RosInterfaceNode::actionHandleCancel(
  Action<ActType>::SharedPtr action,
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>&)
{
  // キャンセルが完了するまで待機
  const auto future = action->client->async_cancel_goal(action->client_gh);
  future.wait();

  const auto res = future.get();
  if (res->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  else {
    return rclcpp_action::CancelResponse::REJECT;
  }
}

template <typename ActType>
void RosInterfaceNode::actionHandleAccepted(
  Action<ActType>::SharedPtr action,
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& server_gh)
{
  // 別メソッドでフィードバックを発行するために ServerGoalHandle を保存
  action->server_gh = server_gh;

  // アクションが終了するまで待機
  const auto future = action->client->async_get_result(action->client_gh);
  future.wait();

  // 結果を返す
  const auto res = future.get();
  switch (res.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      server_gh->succeed(res.result);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      server_gh->canceled(res.result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      server_gh->abort(res.result);
      break;
    default:
      TOBAS_ERROR("Invalid action result code: ", (int)res.code);
      server_gh->abort(res.result);
      break;
  }

  // GoalHandle を初期化
  action->server_gh.reset();
  action->client_gh.reset();
}

int main(int argc, char* argv[])
{
  constexpr long kDefaultNumThreads = 4;
  constexpr long kMinNumThreads = 2;

  rclcpp::init(argc, argv);

  const auto node = std::make_shared<RosInterfaceNode>();

  long num_threads = kDefaultNumThreads;
  if (node->has_parameter("num_threads")) {
    num_threads = node->get_parameter("num_threads").as_int();
  }

  if (num_threads < kMinNumThreads) {
    RCLCPP_WARN_STREAM(
      node->get_logger(),
      "To avoid deadlock with recursive service calls, at least " << kMinNumThreads << " threads are required.");
    num_threads = kMinNumThreads;
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "The number of threads is set to " << num_threads << ".");

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), num_threads);
  exec.add_node(node);

  exec.spin();
}
