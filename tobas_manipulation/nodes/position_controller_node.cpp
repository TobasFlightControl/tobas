#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_std_tools/zip.hpp>
#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_kdl/treeactivejointsextractor.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs_adapter/LinkStateArray.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

using namespace std;

class PositionControllerNode : public tobas::BaseNode
{
  using self = PositionControllerNode;
  using super = tobas::BaseNode;

public:
  explicit PositionControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;

  bool is_initialized_ = false;
  bool is_commanded_ = false;
  sensor_msgs::msg::JointState home_js_;
  rclcpp::Time t_last_cmd_;

  sensor_msgs::msg::JointState::ConstSharedPtr tar_js_;
  tobas_msgs::LinkStateArray::ConstSharedPtr tar_ls_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> positions_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<sensor_msgs::msg::JointState> cur_js_sub_;
  ros2::SubscriberPtr<sensor_msgs::msg::JointState> tar_js_sub_;
  ros2::SubscriberPtr<tobas_msgs::LinkStateArray> tar_ls_sub_;

  void initialize();

  bool jointSpaceControl(tobas_msgs::msg::JointCommandArray& positions_msg);
  bool taskSpaceControl(tobas_msgs::msg::JointCommandArray& positions_msg);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void currentJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& cur_js);
  void targetJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls);
};

PositionControllerNode::PositionControllerNode(const rclcpp::NodeOptions& options)
  : super("position_controller", options)
{
  // Register publishers
  positions_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(tobas::kJointPositionsCmdTopic);

  // Register subscribers
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true);
  cur_js_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::currentJointStateCb, this);
  tar_js_sub_ = createSubscriber(tobas::kPosCtrlJSTopic, &self::targetJointStateCb, this);
  tar_ls_sub_ = createSubscriber(tobas::kPosCtrlLSTopic, &self::targetLinkStateCb, this);
}

void PositionControllerNode::initialize()
{
  // 位置指令タイプの関節のホームポジションを取得
  for (const auto& [jnt_name, jnt_cfg] : drone_.joints)
  {
    if (jnt_cfg.interface != tobas::joint_interface_t::POSITION)
      continue;
    home_js_.name.push_back(jnt_name);
    home_js_.position.push_back(jnt_cfg.home_pos);
    home_js_.velocity.push_back(0.);
    home_js_.effort.push_back(0.);
  }

  // ホームポジションを初期目標状態に設定
  if (home_js_.name.size() > 0)
    tar_js_ = std::make_shared<sensor_msgs::msg::JointState>(home_js_);

  is_initialized_ = true;
}

bool PositionControllerNode::jointSpaceControl(tobas_msgs::msg::JointCommandArray& positions_msg)
{
  // 位置コマンドをそのまま流すだけ
  for (const auto& [name, pos] : tobas_std::zip(tar_js_->name, tar_js_->position))
  {
    positions_msg.commands.emplace_back();
    positions_msg.commands.back().name = name;
    positions_msg.commands.back().data = pos;
  }

  return true;
}

bool PositionControllerNode::taskSpaceControl(tobas_msgs::msg::JointCommandArray&)
{
  TOBAS_ERROR("Task space control of joint position controller is not implemented.");  // TODO

  return true;
}

void PositionControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = *drone;
  initialize();
}

void PositionControllerNode::currentJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr&)
{
  if (!is_initialized_)
    return;

  if (tar_js_ == nullptr && tar_ls_ == nullptr)
    return;

  const auto time_after_last_cmd = (get_clock()->now() - t_last_cmd_).seconds();
  if (is_commanded_ && time_after_last_cmd > tobas::kAutoResetTimeThreshold)
  {
    tar_js_ = std::make_shared<sensor_msgs::msg::JointState>(home_js_);
    tar_ls_ = nullptr;
    is_commanded_ = false;
    TOBAS_WARN(
      "The target joint states are automatically reset because ", tobas::kAutoResetTimeThreshold,
      " seconds have elapsed since the last command.");
  }

  // Create joint velocities command
  auto positions_msg = std::make_unique<tobas_msgs::msg::JointCommandArray>();

  // Joint space control or Task space control
  if (tar_js_ != nullptr)
  {
    if (!jointSpaceControl(*positions_msg))
      return;
  }
  else if (tar_ls_ != nullptr)
  {
    if (!taskSpaceControl(*positions_msg))
      return;
  }
  else
  {
    TOBAS_ERROR("Both target joint state and target link state are NULL.");
    return;
  }

  // Publish joint velocities command
  positions_pub_->publish(move(positions_msg));
}

void PositionControllerNode::targetJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& tar_js)
{
  tar_js_ = tar_js;
  tar_ls_ = nullptr;

  t_last_cmd_ = get_clock()->now();
  is_commanded_ = true;
}

void PositionControllerNode::targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls)
{
  tar_ls_ = tar_ls;
  tar_js_ = nullptr;

  t_last_cmd_ = get_clock()->now();
  is_commanded_ = true;
}

RCLCPP_COMPONENTS_REGISTER_NODE(PositionControllerNode)
