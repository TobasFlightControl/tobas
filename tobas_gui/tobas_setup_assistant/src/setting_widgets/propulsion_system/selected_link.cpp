#include "tobas_setup_assistant/setting_tabs/propulsion_system/selected_link.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
SelectedLinkWidget::SelectedLinkWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot, const QString& link_name)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto button_cols = new QHBoxLayout();
  rows->addLayout(button_cols);

  copy_from_left_button_ = new QPushButton("Copy From Left");
  copy_from_left_button_->setFixedSize(kButtonWidth, kButtonHeight);
  button_cols->addWidget(copy_from_left_button_);

  copy_to_all_button_ = new QPushButton("Copy To All");
  copy_to_all_button_->setFixedSize(kButtonWidth, kButtonHeight);
  button_cols->addWidget(copy_to_all_button_);

  button_cols->addStretch();

  tabs_ = new qt::TabWidget();
  tabs_->ignoreWheelEvent();
  tabs_->setSize(kTabWidth, kTabHeight);
  rows->addWidget(tabs_);

  general_ = new GeneralWidget(robot, link_name);
  esc_ = new ESCWidget();
  motor_ = new MotorWidget();
  propeller_ = new PropellerWidget();
  aerodynamics_ = new AerodynamicsWidget(node, propeller_);
  speed_limit_ = new SpeedLimitWidget(motor_, aerodynamics_);

  tabs_->addTab(general_, general_->name());
  tabs_->addTab(esc_, esc_->name());
  tabs_->addTab(motor_, motor_->name());
  tabs_->addTab(propeller_, propeller_->name());
  tabs_->addTab(aerodynamics_, aerodynamics_->name());
  tabs_->addTab(speed_limit_, speed_limit_->name());

  rows->addStretch();

  // Connection
  connect(copy_to_all_button_, &QPushButton::clicked, [this]() { Q_EMIT copyToAllButtonClicked(); });
  connect(copy_from_left_button_, &QPushButton::clicked, [this]() { Q_EMIT copyFromLeftButtonClicked(); });
  connect(general_, &GeneralWidget::channelChanged, [this](int channel) { Q_EMIT channelChanged(channel); });
  connect(general_, &GeneralWidget::isTiltStateChanged, [this](bool is_tilt) { Q_EMIT isTiltStateChanged(is_tilt); });
  connect(
    general_, &GeneralWidget::tiltJointNameChanged,
    [this](const QString& joint_name) { Q_EMIT tiltJointNameChanged(joint_name); });
}

bool SelectedLinkWidget::isValid()
{
  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto widget = qobject_cast<BaseSelectedLinkSettingWidget*>(tabs_->widget(i));
    if (!widget->isValid())
      return false;
  }

  return true;
}

void SelectedLinkWidget::copyFrom(const SelectedLinkWidget* src)
{
  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto des_widget = qobject_cast<BaseSelectedLinkSettingWidget*>(tabs_->widget(i));
    const auto src_widget = qobject_cast<BaseSelectedLinkSettingWidget*>(src->tabs_->widget(i));
    des_widget->copyFrom(src_widget);
  }
}

YAML::Node SelectedLinkWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto widget = qobject_cast<BaseSelectedLinkSettingWidget*>(tabs_->widget(i));
    node[widget->name()] = widget->dump();
  }

  return node;
}

void SelectedLinkWidget::load(const YAML::Node& node)
{
  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto widget = qobject_cast<BaseSelectedLinkSettingWidget*>(tabs_->widget(i));
    widget->load(node[widget->name()]);
  }
}

const GeneralWidget* SelectedLinkWidget::general() const
{
  return general_;
}

const ESCWidget* SelectedLinkWidget::esc() const
{
  return esc_;
}

const MotorWidget* SelectedLinkWidget::motor() const
{
  return motor_;
}

const PropellerWidget* SelectedLinkWidget::propeller() const
{
  return propeller_;
}

const AerodynamicsWidget* SelectedLinkWidget::aerodynamics() const
{
  return aerodynamics_;
}

const SpeedLimitWidget* SelectedLinkWidget::speedLimit() const
{
  return speed_limit_;
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
