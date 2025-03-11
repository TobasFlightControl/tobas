#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

class DisturbanceObserverNode : public tobas::BaseNode
{
  using self = DisturbanceObserverNode;
  using super = tobas::BaseNode;

public:
  explicit DisturbanceObserverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Dynamic parameters
  double cutoff_freq_;  // [Hz]

  kdl::Tree tree_;
  tobas::Drone drone_;

  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;
  tobas::TreeJointStateConverter js_converter_;

  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::RotorStateArray::ConstSharedPtr rotor_states_;
  bool js_received_ = false;

  dsp::LowPassFilter<kdl::Vector> force_lpf_;
  dsp::LowPassFilter<kdl::Vector> torque_lpf_;

  ros2::PublisherPtr<tobas_kdl_msgs::WrenchStamped> dist_force_pub_;

  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> rotor_states_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> joint_states_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;

  bool cutoffFreqCb(const long& p);

  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& rotor_states);
  void jointStatesCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& joint_states);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
};

DisturbanceObserverNode::DisturbanceObserverNode(const rclcpp::NodeOptions& options)
  : super("disturbance_observer", options), fk_solver_(tree_), inertia_solver_(tree_), js_converter_(tree_)
{
  addDynamicIntParam("cutoff_frequency", &self::cutoffFreqCb, this, 10, 1, 100);

  dist_force_pub_ = createPublisher<tobas_kdl_msgs::WrenchStamped>(tobas::kDisturbanceForceTopic);

  tree_sub_ = createSubscriber(tobas::kKdlTreeTopic, &self::treeCb, this, true, true);
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  rotor_states_sub_ = createSubscriber(tobas::kRotorStatesTopic, &self::rotorStatesCb, this);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
}

bool DisturbanceObserverNode::cutoffFreqCb(const long& p)
{
  if (!force_lpf_.setCutoffFrequency(p))
  {
    TOBAS_ERROR("Failed to set cutoff frequency of force LPF.");
    return false;
  }

  if (!torque_lpf_.setCutoffFrequency(p))
  {
    TOBAS_ERROR("Failed to set cutoff frequency of torque LPF.");
    return false;
  }

  return true;
}

void DisturbanceObserverNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;

  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  js_converter_.updateInternalDataStructures();
}

void DisturbanceObserverNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = *drone;

  odom_ = nullptr;
  rotor_states_ = nullptr;
  js_received_ = false;

  if (drone->hasServoJoint())
    joint_states_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::jointStatesCb, this);
  else
    joint_states_sub_ = nullptr;
}

void DisturbanceObserverNode::rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& rotor_states)
{
  rotor_states_ = rotor_states;
}

void DisturbanceObserverNode::jointStatesCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& joint_states)
{
  if (js_converter_.convert(*joint_states) < 0)
  {
    TOBAS_ERROR("Joint state converter failed: ", js_converter_.errorMessage());
    return;
  }

  js_received_ = true;
}

void DisturbanceObserverNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  if (tree_.getNrOfJoints() == 0)
    return;

  if (drone_.prop->numRotors() == 0)
    return;

  if (rotor_states_ == nullptr)
    return;

  if (joint_states_sub_ != nullptr && !js_received_)
    return;

  if (odom_ == nullptr)
  {
    force_lpf_.setValue(kdl::Vector::Zero());
    torque_lpf_.setValue(kdl::Vector::Zero());

    odom_ = odom;
    return;
  }

  // サンプリング時間を計算
  const auto dt = (odom->header.stamp - odom_->header.stamp).seconds();
  odom_ = odom;

  // 順運動学を計算
  if (fk_solver_.JntToCart(js_converter_.getPosition()) < 0)
  {
    TOBAS_ERROR("Forward kinematics failed: ", fk_solver_.errorMessage());
    return;
  }

  // 質量特性を計算
  if (inertia_solver_.JntToCart(js_converter_.getPosition()) < 0)
  {
    TOBAS_ERROR("Inertia solver failed: ", inertia_solver_.errorMessage());
    return;
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();
  const auto& mass = inertia.getMass();
  const auto weight = mass * tobas_std::kGravity;

  // 推力がかかる項を計算
  kdl::Vector trans_sum = kdl::Vector::Zero();
  kdl::Vector rot_sum = kdl::Vector::Zero();
  for (const auto& rotor_state : rotor_states_->states)
  {
    if (rotor_state.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE)
    {
      TOBAS_WARN_THROTTLE(
        tobas::kTypicalWarnPeriod, "No communication with rotor \"", rotor_state.link_name,
        "\". Its rotation speed is estimated to 0.");
    }
    else
    {
      const auto& rotor = drone_.prop->rotors.at(rotor_state.link_name);

      const auto elem = tree_.getSegment(rotor->link_name)->second;
      const auto& B_Rot_Par = fk_solver_.getFrame(elem.parent->first).M;
      const auto axis_B = B_Rot_Par * elem.segment.joint().axis();

      const auto d = rotor->sign();
      const auto& cm = rotor->moment_const;
      const auto& B_Pos_B2P = fk_solver_.getFrame(rotor->link_name).p;
      const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;

      trans_sum += axis_B * rotor_state.thrust;
      rot_sum += (B_Pos_G2P * axis_B - (d * cm) * axis_B) * rotor_state.thrust;
    }
  }

  // 外力を計算
  // TODO: 固定翼の力も考慮
  const auto& W_Rot_B = odom->frame.M;
  const auto& gyro_B = odom->twist.rot;
  const auto& acc_B = odom->accel.linear;
  const auto& dgyro_B = odom->accel.angular;  // FIXME: DGyroの数値誤差が大きく外力トルクの推定制度が低い
  const auto force_W = kdl::Vector(0, 0, weight) + W_Rot_B * (mass * acc_B - trans_sum);
  const auto torque_B = I_B * dgyro_B + gyro_B * (I_B * gyro_B) - rot_sum;

  // 外力をLPFに通す
  force_lpf_.update(force_W, dt);
  torque_lpf_.update(torque_B, dt);

  // 外力メッセージを作成
  auto dist_force_msg = std::make_unique<tobas_kdl_msgs::WrenchStamped>();
  dist_force_msg->header.stamp = odom->header.stamp;
  dist_force_msg->wrench.force = force_lpf_.getValue();
  dist_force_msg->wrench.torque = torque_lpf_.getValue();

  // 外力メッセージを発行
  dist_force_pub_->publish(std::move(dist_force_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(DisturbanceObserverNode)
