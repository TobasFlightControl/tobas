#include <QPushButton>

#include <tobas_std_tools/vector.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_setup_assistant/joint_state_publisher.hpp"

using namespace std;

namespace gui
{
namespace setup_assistant
{
JointStatePublisherWidget::JointStatePublisherWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot)
  : node_(node), robot_(robot)
{
  rows_ = new QVBoxLayout();
  slider_rows_ = new qt::ScrollableVBoxLayout();
  button_rows_ = new QVBoxLayout();

  setLayout(rows_);
  rows_->addLayout(slider_rows_);
  rows_->addLayout(button_rows_);

  auto center_button = new QPushButton("Center");
  connect(center_button, &QPushButton::clicked, this, &self::onCenterButtonClicked);
  button_rows_->addWidget(center_button);

  rows_->addStretch();

  // Register publishers
  js_pub_ = ros2::createPublisher<sensor_msgs::msg::JointState>(node_, tobas::kJointStatesTopic);
  drs_pub_ = ros2::createPublisher<moveit_msgs::msg::DisplayRobotState>(node_, "display_robot_state");

  // Create timers
  publish_timer_ = ros2::createTimer(node_, 100ms, &self::publish, this);
}

void JointStatePublisherWidget::onRobotLoaded()
{
  js_.name.clear();
  js_.position.clear();
  sliders_.clear();
  qt::clearLayout(slider_rows_);

  for (const auto& [seg_name, seg_ele] : robot_.tree().getSegments())
  {
    const auto& joint = seg_ele.segment.joint();
    if (joint.type == kdl::Joint::Fixed)
      return;

    js_.name.push_back(joint.name);
    js_.position.push_back(0.);

    auto slider = new qt::DoubleSliderDisplay();
    slider->setText(QString::fromStdString(joint.name));
    slider->setMinimum(joint.lower_limit);
    slider->setMaximum(joint.upper_limit);
    slider->setValue(0.);
    connect(
      slider, &qt::DoubleSliderDisplay::valueChanged, this,
      bind(&self::onValueChanged, this, placeholders::_1, joint.name));

    sliders_.push_back(slider);
    slider_rows_->addWidget(slider);
  }
}

void JointStatePublisherWidget::onValueChanged(double value, const string& jnt_name)
{
  const auto idx = tobas_std::index(js_.name, jnt_name);
  if (idx < 0)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Joint \"" << jnt_name << "\" does not exist.");
    return;
  }

  js_.position.at(idx) = value;
}

void JointStatePublisherWidget::onCenterButtonClicked()
{
  for (auto& slider : sliders_)
    slider->setCenterValue();
}

void JointStatePublisherWidget::publish()
{
  js_.header.stamp = node_->get_clock()->now();

  auto js = make_unique<sensor_msgs::msg::JointState>(js_);
  js_pub_->publish(std::move(js));

  auto drs = make_unique<moveit_msgs::msg::DisplayRobotState>();
  drs->state.joint_state = js_;
  drs_pub_->publish(std::move(drs));
}
}  // namespace setup_assistant
}  // namespace gui
