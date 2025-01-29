#include <QPushButton>
#include <QHBoxLayout>

#include <tobas_std_tools/vector.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_setup_assistant/joint_state_publisher.hpp"

using namespace std;

namespace gui
{
namespace setup_assistant
{
JointStatePublisherWidget::JointStatePublisherWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot)
  : node_(node), robot_(robot), rnd_gen_(rnd_dev_())
{
  slider_rows_ = new QVBoxLayout();

  const auto scroll_area = new qt::ScrollArea();
  scroll_area->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  scroll_area->setLayout(slider_rows_);

  const auto center_button = new QPushButton("Center");
  const auto random_button = new QPushButton("Random");

  center_button->setFixedHeight(kButtonHeight);
  random_button->setFixedHeight(kButtonHeight);

  // Layout
  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(center_button);
  button_cols->addWidget(random_button);

  const auto rows = new QVBoxLayout();
  rows->addWidget(scroll_area);
  rows->addLayout(button_cols);

  setLayout(rows);

  // Connection
  connect(&robot, &RobotInfo::loaded, this, &self::onRobotLoaded);
  connect(center_button, &QPushButton::clicked, this, &self::onCenterButtonClicked);
  connect(random_button, &QPushButton::clicked, this, &self::onRandomButtonClicked);
  connect(&publish_timer_, &QTimer::timeout, this, &self::publish);

  // Register publishers
  js_pub_ = ros2::createPublisher<sensor_msgs::msg::JointState>(node_, "joint_states");
  drs_pub_ = ros2::createPublisher<moveit_msgs::msg::DisplayRobotState>(node_, "display_robot_state", false, true);
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

void JointStatePublisherWidget::onRobotLoaded()
{
  js_.name.clear();
  js_.position.clear();
  sliders_.clear();
  qt::clearLayout(slider_rows_);

  for (const auto& [_, elem] : robot_.tree().getSegments())
  {
    const auto& joint = elem.segment.joint();
    if (joint.type == kdl::Joint::FIXED)
      continue;

    js_.name.push_back(joint.name);
    js_.position.push_back(0.);

    const auto slider = new qt::DoubleSliderDisplay();
    slider->setText(QString::fromStdString(joint.name));

    auto lower_limit = joint.lower_limit;
    auto upper_limit = joint.upper_limit;
    if (joint.type == kdl::Joint::ROTATION && upper_limit - lower_limit > 2 * M_PI)
    {
      lower_limit = -M_PI;
      upper_limit = +M_PI;
    }
    slider->setMinimum(lower_limit);
    slider->setMaximum(upper_limit);

    slider->setValue(0.);

    connect(
      slider, &qt::DoubleSliderDisplay::valueChanged, this,
      bind(&self::onValueChanged, this, placeholders::_1, joint.name));

    sliders_.push_back(slider);
    slider_rows_->addWidget(slider);

    RCLCPP_DEBUG_STREAM(node_->get_logger(), "\"" << joint.name << "\" is added to the JSP slider.");
  }

  slider_rows_->addStretch();

  // Start to publish joint states
  publish_timer_.start(100);
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
  {
    const auto value = (slider->getMinimum() + slider->getMaximum()) / 2;
    slider->setValue(value);
  }
}

void JointStatePublisherWidget::onRandomButtonClicked()
{
  for (auto& slider : sliders_)
  {
    uniform_real_distribution<double> uniform(slider->getMinimum(), slider->getMaximum());
    const auto value = uniform(rnd_gen_);
    slider->setValue(value);
  }
}
}  // namespace setup_assistant
}  // namespace gui
